# laptop_ai/messaging_client.py
import asyncio
import json
import websockets
from laptop_ai.config import VPS_WS, AUTH_TOKEN
import time

class MessagingClient:
    """
    Robust websocket client that auto-reconnects and does a handshake:
    sends {"id": "<client_id>", "token": AUTH_TOKEN} immediately after connect.
    """

    def __init__(self, client_id="laptop_vision"):
        self.client_id = client_id
        self.ws = None
        self.connected = False
        self._recv_handlers = []
        self._send_lock = asyncio.Lock()
        self._closing = False
        self.last_msg_ts = 0.0
        self._reconnecting = False
        self._watchdog_started = False

    async def connect(self):
        # Single-flight: don't let multiple callers open parallel sockets.
        if self._reconnecting:
            return
        self._reconnecting = True
        backoff = 1.0
        try:
            while not self._closing:
                try:
                    print("CONNECTING TO VPS_WS =", VPS_WS)
                    # ping_interval=None: DISABLE the keepalive ping. The heavy GPU vision loop
                    # periodically blocks this event loop >10s, so a keepalive ping would time out
                    # and self-close the link (1011) every few seconds — tearing down the command
                    # channel so cmd_vel never reached the FC. We instead rely on the data-staleness
                    # watchdog (no telemetry for >20s = truly dead) to reconnect. Telemetry flows
                    # ~10Hz when the loop runs, so a real drop is still caught quickly.
                    self.ws = await websockets.connect(
                        VPS_WS, ping_interval=None, ping_timeout=None,
                        close_timeout=5, open_timeout=12, max_size=None)
                    await self.ws.send(json.dumps({"id": self.client_id, "token": AUTH_TOKEN}))
                    self.connected = True
                    self.last_msg_ts = time.time()
                    print("MessagingClient connected")
                    asyncio.create_task(self._recv_loop())
                    if not self._watchdog_started:
                        self._watchdog_started = True
                        asyncio.create_task(self._watchdog())
                    return
                except Exception as e:
                    print("WS connect err:", e)
                    await asyncio.sleep(backoff)
                    backoff = min(15.0, backoff * 2.0)
        finally:
            self._reconnecting = False

    async def _recv_loop(self):
        try:
            while True:
                raw = await self.ws.recv()
                self.last_msg_ts = time.time()
                try:
                    packet = json.loads(raw)
                except Exception:
                    packet = {"raw": raw}
                for h in list(self._recv_handlers):
                    try:
                        await h(packet)
                    except Exception as e:
                        print("Handler error:", e)
        except Exception as e:
            print("WS recv failed:", e)
            self.connected = False   # the watchdog will reconnect

    async def _watchdog(self):
        """Self-heal: force a reconnect if the link drops OR goes silent (no msg for 15s)."""
        while not self._closing:
            await asyncio.sleep(5.0)
            stale = (time.time() - self.last_msg_ts) > 20.0
            if not self.connected or stale:
                print(f"🔁 MessagingClient watchdog: reconnecting (connected={self.connected}, stale={stale})")
                self.connected = False
                try:
                    if self.ws:
                        await self.ws.close()
                except Exception:
                    pass
                await self.connect()

    def add_recv_handler(self, coro):
        self._recv_handlers.append(coro)

    async def send(self, packet: dict):
        async with self._send_lock:
            if not self.connected:
                await self.connect()
            tries = 0
            while tries < 3:
                try:
                    await self.ws.send(json.dumps(packet))
                    return
                except Exception as e:
                    print("WS send failed:", e)
                    self.connected = False
                    tries += 1
                    await asyncio.sleep(0.5)
                    await self.connect()

    async def close(self):
        self._closing = True
        if self.ws:
            await self.ws.close()
