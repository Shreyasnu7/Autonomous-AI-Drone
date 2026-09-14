# File: radxa_agent/radxa_agent.py
# SAFE MAVLINK EXECUTOR FOR RADXA ZERO
# Receives AI motion primitives and converts them to SAFE setpoints.
# AUTOPILOT remains FULL authority: arming, takeoff, landing, motor mixing.

import asyncio
import json
import time
import math
import os
from websockets import connect
from datetime import datetime

# ----------------------------------------
# CONFIG
# ----------------------------------------
from laptop_ai.motion_curve import BezierCurve  # curve math only – safe
from laptop_ai.obstacle_warp import ObstacleWarp

VPS_WS = "wss://drone-server-r0qe.onrender.com/ws/connect/RADXA_X"
AUTH_TOKEN = "bearer_token"

# Local bridge WebSocket (real_bridge_service.py runs on same Radxa)
LOCAL_BRIDGE_WS = "ws://127.0.0.1:8000"

LOG_PATH = "/tmp/radxa_agent_log.txt"

# Safety limits
MAX_SPEED = 4.0            # m/s
MAX_CLIMB_RATE = 2.0       # m/s
MAX_DESCENT_RATE = 1.2     # m/s
MAX_YAW_RATE = 45.0        # deg/s
MAX_DISTANCE = 150.0       # meters from home
MIN_BATTERY = 0.22         # 22%
MIN_GPS_SATS = 7

# ----------------------------------------
# MOCK TELEMETRY (replace with actual read functions)
# ----------------------------------------
def get_battery():
    # placeholder: replace with actual MAVLink telemetry read
    return 0.95

def get_gps_sats():
    return 12

def get_position():
    return [0.0, 0.0, 5.0]  # assume 5m altitude

def get_heading():
    return 0.0

def rc_override_active():
    return False


# ----------------------------------------
# SAFE EXECUTOR CLASS
# ----------------------------------------
class RadxaExecutor:

    def __init__(self):
        self.ws = None
        self.bridge_ws = None  # Local bridge connection
        self.connected = False
        self.last_setpoint_time = 0
        self.keepalive_period = 0.4
        self.active_curve = None
        self.curve_start_time = 0
        self.warp_engine = ObstacleWarp()

    # ----------------------------------------
    # Log helper
    # ----------------------------------------
    def log(self, msg):
        line = f"[{datetime.now()}] {msg}\n"
        print(line.strip())
        with open(LOG_PATH, "a") as f:
            f.write(line)

    # ----------------------------------------
    # WebSocket Handling
    # ----------------------------------------
    async def connect_ws(self):
        while True:
            try:
                self.log(f"Connecting WS → {VPS_WS}")
                self.ws = await connect(VPS_WS)
                await self.ws.send(json.dumps({
                    "id": "radxa",
                    "token": AUTH_TOKEN
                }))
                self.connected = True
                self.log("WS connected.")
                asyncio.create_task(self.recv_loop())
                return
            except Exception as e:
                self.log(f"WS connect fail: {e}")
                await asyncio.sleep(2)

    async def connect_bridge(self):
        """Connect to the local bridge that owns the FC serial port."""
        while True:
            try:
                self.log(f"Connecting local bridge -> {LOCAL_BRIDGE_WS}")
                self.bridge_ws = await connect(LOCAL_BRIDGE_WS)
                await self.bridge_ws.send(json.dumps({"id": "radxa_agent", "token": AUTH_TOKEN}))
                self.log("Local bridge connected.")
                return
            except Exception as e:
                self.log(f"Bridge connect fail: {e}. Retrying...")
                await asyncio.sleep(2)

    async def _send_bridge(self, msg):
        """Send a message to local bridge, reconnect if needed."""
        try:
            if self.bridge_ws:
                await self.bridge_ws.send(msg)
        except Exception as e:
            self.log(f"Bridge send fail: {e}")
            self.bridge_ws = None
            await self.connect_bridge()

    async def recv_loop(self):
        while True:
            try:
                msg = await self.ws.recv()
                packet = json.loads(msg)
                await self.handle_packet(packet)
            except Exception as e:
                self.log(f"WS recv fail: {e}")
                self.connected = False
                await self.connect_ws()
                break

    # ----------------------------------------
    # PACKET HANDLING
    # ----------------------------------------
    async def handle_packet(self, packet):
        if packet.get("type") == "ai_plan":
            await self.apply_primitive(packet["primitive"])

    # ----------------------------------------
    # SAFETY GATES
    # ----------------------------------------
    def safety_ok(self):
        if rc_override_active():
            self.log("RC override active → BLOCK AI")
            return False

        if get_battery() < MIN_BATTERY:
            self.log("Low battery → BLOCK AI")
            return False

        if get_gps_sats() < MIN_GPS_SATS:
            self.log("Weak GPS → BLOCK AI")
            return False

        return True

    # ----------------------------------------
    # APPLY AI PRIMITIVE
    # ----------------------------------------
    async def apply_primitive(self, prim):
        """
        Prim example:
        {
          "action": "MOVE",
          "params": {"x": 3, "y": -1, "z": 6, "speed": 3.5}
        }
        """

        if not self.safety_ok():
            return

        action = prim.get("action", "HOVER")
        p = prim.get("params", {})

        if action == "HOVER":
            await self.send_hover()
            return

        if action == "MOVE":
            await self.safe_move(p)
            return

        if action == "CURVE":
            await self.start_curve_motion(p)
            return

        if action == "YAW":
            await self.safe_yaw(p)
            return

        if action == "FOCUS":
            await self.safe_focus(p)
            return

    # ----------------------------------------
    # PRIMITIVE EXECUTORS
    # ----------------------------------------
    async def safe_move(self, p):
        target = [
            float(p.get("x", 0)),
            float(p.get("y", 0)),
            float(p.get("z", 5))
        ]
        spd = min(float(p.get("speed", MAX_SPEED)), MAX_SPEED)

        self.log(f"AI MOVE → {target} @ {spd} m/s")

        self.send_mavlink_setpoint(
            x=target[0], y=target[1], z=target[2],
            vx=0, vy=0, vz=0
        )

    async def safe_yaw(self, p):
        rate = min(abs(p.get("rate", 0)), MAX_YAW_RATE)
        direction = 1 if p.get("rate", 0) >= 0 else -1
        self.log(f"AI YAW → {direction * rate} deg/s")

        self.send_mavlink_setpoint(
            yaw_rate=direction * math.radians(rate)
        )

    async def safe_focus(self, p):
        """Point drone toward object without moving it."""
        fx, fy, fz = p.get("x", 0), p.get("y", 0), p.get("z", 0)
        pos = get_position()
        vec = [fx - pos[0], fy - pos[1], fz - pos[2]]
        yaw = math.atan2(vec[1], vec[0])

        self.send_mavlink_setpoint(yaw=yaw)

    async def start_curve_motion(self, p):
        """Start streaming warped Bezier curve."""
        pts = p.get("points")
        if not pts:
            return

        # Build curve
        P = [list(map(float, q)) for q in pts]
        curve = BezierCurve(P[0], P[1], P[2], P[3])

        # Warp around obstacles (mock data for now)
        obstacles = p.get("obstacles", [])
        final_curve = self.warp_engine.warp_curve(curve, obstacles)

        self.active_curve = final_curve
        self.curve_start_time = time.time()
        asyncio.create_task(self.stream_curve())

    async def stream_curve(self):
        duration = 4.0
        self.log("Streaming cinematic curve…")

        while True:
            t = (time.time() - self.curve_start_time) / duration
            if t >= 1.0:
                break
            pt = self.active_curve.point(max(0, min(t, 1)))
            self.send_mavlink_setpoint(
                x=float(pt[0]), y=float(pt[1]), z=float(pt[2])
            )
            await asyncio.sleep(0.05)

        self.log("Curve complete.")
        self.active_curve = None

    # ----------------------------------------
    # MAVLINK SETPOINT — routes through local bridge
    # ----------------------------------------
    def send_mavlink_setpoint(self, x=None, y=None, z=None,
                              vx=None, vy=None, vz=None,
                              yaw=None, yaw_rate=None):
        """
        Sends setpoint to the local bridge (real_bridge_service.py)
        which owns the FC serial connection. The bridge converts
        these numbers into real MAVLink commands.
        """
        # Build params dict with only non-None values
        params = {}
        if vx is not None or vy is not None or vz is not None:
            params['vx'] = vx or 0
            params['vy'] = vy or 0
            params['vz'] = vz or 0
        if x is not None or y is not None or z is not None:
            params['x'] = x or 0
            params['y'] = y or 0
            params['z'] = z or 0
        if yaw is not None:
            params['yaw'] = math.degrees(yaw)
        if yaw_rate is not None:
            params['yaw_rate'] = math.degrees(yaw_rate)

        self.log(f"[SETPOINT] {params}")

        # Send to local bridge via WebSocket
        if self.bridge_ws:
            msg = json.dumps({
                "type": "ai_plan",
                "payload": {
                    "action": "SETPOINT",
                    "params": params
                }
            })
            asyncio.ensure_future(self._send_bridge(msg))

    async def send_hover(self):
        self.send_mavlink_setpoint(
            vx=0, vy=0, vz=0
        )

    # ----------------------------------------
    # MAIN LOOP
    # ----------------------------------------
    async def run(self):
        await self.connect_bridge()
        await self.connect_ws()

        while True:
            # Keep-alive for offboard safety
            if self.connected:
                self.send_hover()
            await asyncio.sleep(self.keepalive_period)


# ----------------------------------------
# ENTRYPOINT
# ----------------------------------------
if __name__ == "__main__":
    exe = RadxaExecutor()
    asyncio.run(exe.run())
