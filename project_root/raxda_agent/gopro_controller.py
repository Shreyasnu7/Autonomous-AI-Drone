# radxa_agent/gopro_controller.py
import asyncio
import aiohttp
import time

class GoProController:
    def __init__(self, ip="10.5.5.9"):  # default GoPro WiFi IP
        self.base = f"http://{ip}"
        self.session = None

    async def connect(self):
        self.session = aiohttp.ClientSession()
        # optionally test connection
        try:
            async with self.session.get(self.base, timeout=2) as r:
                if r.status == 200:
                    return True
        except Exception:
            return False

    async def start_recording(self):
        # endpoint varies by firmware - this is an example for older models
        try:
            async with self.session.get(f"{self.base}/gp/gpControl/command/shutter?p=1") as r:
                return r.status == 200
        except Exception:
            return False

    async def stop_recording(self):
        try:
            async with self.session.get(f"{self.base}/gp/gpControl/command/shutter?p=0") as r:
                return r.status == 200
        except Exception:
            return False

    async def set_mode(self, mode_id:int):
        # 0 photo, 1 video etc.
        await self.session.get(f"{self.base}/gp/gpControl/command/mode?p={mode_id}")

    async def set_setting(self, setting_name, value):
        # GoPro REST settings endpoints differ by FW; use official SDK or extend this.
        pass

    async def close(self):
        if self.session:
            await self.session.close()
            self.session = None
