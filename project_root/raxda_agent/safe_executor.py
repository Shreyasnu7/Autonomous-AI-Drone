# File: radxa_agent/safe_executor.py

from radxa_agent.mavlink_adapter import MavlinkAdapter

class SafeExecutor:
    def __init__(self):
        self.mav = MavlinkAdapter()

    async def start(self):
        await self.mav.connect()

    async def stream_curve_point(self, point, yaw=None):
        """Stream a single Bezier curve point to the FC."""
        await self.mav.set_position(point[0], point[1], point[2], yaw_deg=(yaw or 0))
        await self.mav.keepalive()



    async def execute_primitive(self, primitive):
        """
        primitive example:
        {
           "action": "MOVE_TO",
           "params": {"x": 5, "y": -2, "z": 3, "yaw": 45}
        }
        """
        action = primitive.get("action")
        p = primitive.get("params", {})

        if action == "MOVE_TO":
            await self.mav.set_position(
                p.get("x", 0),
                p.get("y", 0),
                p.get("z", 2),
                yaw_deg=p.get("yaw", 0)
            )

        elif action == "VELOCITY":
            await self.mav.set_velocity(
                p.get("vx", 0),
                p.get("vy", 0),
                p.get("vz", 0),
                yaw_deg=p.get("yaw", 0)
            )

        elif action == "YAW_ONLY":
            await self.mav.set_yaw(p.get("yaw", 0))

        elif action == "YAW_RATE":
            await self.mav.set_yaw_rate(p.get("rate", 0))

        # ALWAYS send keepalive after each primitive
        await self.mav.keepalive()
