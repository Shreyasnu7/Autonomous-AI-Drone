# radxa_agent/ws_client.py
import asyncio, json, time
import websockets
from config import WS_URL, AUTH_TOKEN

class SafeWSClient:
    def __init__(self, client_id="radxa"):
        self.client_id = client_id
        self.ws = None

    async def connect(self):
        backoff = 1
        while True:
            try:
                self.ws = await websockets.connect(WS_URL)
                # send auth/hello
                await self.ws.send(json.dumps({"id": self.client_id, "token": AUTH_TOKEN}))
                print("✅ WS connected to server")
                return
            except Exception as e:
                print("⚠️ WS connect failed:", e)
                await asyncio.sleep(backoff)
                backoff = min(30, backoff * 2)

    async def send(self, packet):
        if self.ws is None:
            await self.connect()
        try:
            await self.ws.send(json.dumps(packet))
        except Exception:
            print("⚠️ WS send failed, reconnecting...")
            await self.connect()

    async def recv_loop(self, handler):
        # handler is async function(packet)
        while True:
            try:
                msg = await self.ws.recv()
                packet = json.loads(msg)
                await handler(packet)
            except Exception as e:
                print("⚠️ WS receive error:", e)
                await self.connect()
