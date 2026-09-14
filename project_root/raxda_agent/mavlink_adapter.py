# File: radxa_agent/mavlink_adapter.py
# SAFE MAVLINK SETPOINT ADAPTER
# Converts high-level setpoints into MAVSDK offboard commands.
# Autopilot remains in full control of stabilization, safety, limits.

import asyncio
from mavsdk import System
from mavsdk.offboard import (
    OffboardError,
    PositionNedYaw,
    VelocityNedYaw,
)

class MavlinkAdapter:
    def __init__(self):
        self.drone = System()
        self.ready = False
        self.last_keepalive = 0

    async def connect(self):
        print("Connecting to Flight Controller...")
        # Real hardware: FC connected via serial on Radxa Zero 3W
        # /dev/ttyS2 @ 19200 baud (matches real_bridge_service.py)
        # Falls back to UDP SITL if serial not available (dev/testing)
        import os
        if os.path.exists("/dev/ttyS2"):
            await self.drone.connect(system_address="serial:///dev/ttyS2:19200")
        else:
            print("WARNING: /dev/ttyS2 not found, falling back to UDP SITL")
            await self.drone.connect(system_address="udp://:14540")

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("FC connected.")
                break

        self.ready = True

        # Start offboard mode with a neutral setpoint
        try:
            await self.drone.offboard.start()
        except OffboardError:
            print("Offboard start failed – retrying with neutral setpoint")
            await self._neutral_setpoint()
            await asyncio.sleep(0.2)
            await self.drone.offboard.start()

        print("Offboard mode active.")

    # -----------------------------------------------------
    # LOW-LEVEL SAFE SETPOINT FUNCTIONS
    # -----------------------------------------------------
    async def set_position(self, x, y, z, yaw_deg=None):
        if not self.ready:
            return

        if yaw_deg is None:
            yaw_deg = 0.0

        try:
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(north_m=x, east_m=y, down_m=-z, yaw_deg=yaw_deg)
            )
        except OffboardError as e:
            print(f"[ERR] set_position failed: {e}")

    async def set_velocity(self, vx, vy, vz, yaw_deg=None):
        if not self.ready:
            return

        if yaw_deg is None:
            yaw_deg = 0.0

        try:
            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(vx, vy, vz, yaw_deg)
            )
        except OffboardError as e:
            print(f"[ERR] set_velocity failed: {e}")

    async def set_yaw(self, yaw_deg):
        if not self.ready:
            return

        try:
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(
                    north_m=0, east_m=0, down_m=0,
                    yaw_deg=yaw_deg
                )
            )
        except OffboardError as e:
            print(f"[ERR] set_yaw failed: {e}")

    async def set_yaw_rate(self, yaw_rate_deg_s):
        if not self.ready:
            return

        try:
            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(0, 0, 0, yaw_rate_deg_s)
            )
        except OffboardError as e:
            print(f"[ERR] set_yaw_rate failed: {e}")

    # -----------------------------------------------------
    # KEEPALIVE (OFFBOARD SAFETY)
    # -----------------------------------------------------
    async def keepalive(self):
        """Send neutral setpoint every 0.3 sec so offboard does not timeout."""
        if not self.ready:
            return

        try:
            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(0, 0, 0, 0)
            )
        except OffboardError:
            print("Keepalive lost → attempting restart...")
            await self._neutral_setpoint()
            await asyncio.sleep(0.2)
            try:
                await self.drone.offboard.start()
                print("Offboard resumed.")
            except Exception:
                print("Offboard restart failed.")

    # -----------------------------------------------------
    # INTERNAL: neutral setpoint
    # -----------------------------------------------------
    async def _neutral_setpoint(self):
        """Zero movement, used when restarting offboard."""
        try:
            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(0, 0, 0, 0)
            )
        except Exception:
            pass


# -----------------------------------------------------
# TEST ENTRYPOINT
# -----------------------------------------------------
async def main():
    mav = MavlinkAdapter()
    await mav.connect()
    print("Mavlink Adapter ready.")

    while True:
        await mav.keepalive()
        await asyncio.sleep(0.3)

if __name__ == "__main__":
    asyncio.run(main())
