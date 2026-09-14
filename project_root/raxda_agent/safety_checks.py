# radxa_agent/safety_checks.py
import asyncio, time, json, os
from config import MIN_BATTERY, MIN_SATS

# This module exposes a simple interface:
# - get_status() -> dict with keys: battery, sats, imu_ok, rc_ok, gps_fix
# Implementation will try to use MAVSDK if available, otherwise simulate.

try:
    # optional import, only used for telemetry reading (not actuation)
    from mavsdk import System
    MAVSDK_AVAILABLE = True
except Exception:
    MAVSDK_AVAILABLE = False

# If using MAVSDK, we will expose an async telemetry reader; else we simulate.
class TelemetryReader:
    def __init__(self, use_mavsdk=False, connection_str=None):
        self.use_mavsdk = use_mavsdk and MAVSDK_AVAILABLE
        self.connection_str = connection_str
        self._cached = {
            "battery": 1.0,
            "sats": 10,
            "imu_ok": True,
            "rc_ok": True,
            "gps_fix": True,
            "last_update": time.time()
        }
        self._fc = None

    async def start(self):
        if self.use_mavsdk:
            self._fc = System()
            await self._fc.connect(system_address=self.connection_str)
            print("MAVSDK: connected for telemetry (read-only)")

            async def battery_loop():
                async for bat in self._fc.telemetry.battery():
                    self._cached["battery"] = bat.remaining_percent
                    self._cached["last_update"] = time.time()
            asyncio.create_task(battery_loop())

            async def gps_loop():
                async for gps in self._fc.telemetry.gps_info():
                    self._cached["sats"] = gps.num_satellites
                    self._cached["last_update"] = time.time()
            asyncio.create_task(gps_loop())

            async def health_loop():
                async for hlth in self._fc.telemetry.health():
                    self._cached["imu_ok"] = hlth.is_gyrometer_calibration_ok and hlth.is_accelerometer_calibration_ok
                    self._cached["last_update"] = time.time()
            asyncio.create_task(health_loop())

            # rc_status can be optional
            try:
                async def rc_loop():
                    async for rc in self._fc.telemetry.rc_status():
                        self._cached["rc_ok"] = rc.is_available
                        self._cached["last_update"] = time.time()
                asyncio.create_task(rc_loop())
            except Exception:
                pass
        else:
            # start simulator updates
            asyncio.create_task(self._sim_loop())

    async def _sim_loop(self):
        while True:
            # fake telemetry values; in real usage this is replaced by mavsdk streams above
            self._cached["battery"] = max(0.0, self._cached["battery"] - 0.0001)
            self._cached["sats"] = max(0, self._cached["sats"])
            self._cached["last_update"] = time.time()
            await asyncio.sleep(1.0)

    def get_status(self):
        s = dict(self._cached)
        s["ok_battery"] = s["battery"] >= MIN_BATTERY
        s["ok_sats"] = s["sats"] >= MIN_SATS
        return s
