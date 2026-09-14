# File: radxa_agent/camera_executor.py
# Executes AI camera plans on GoPro + internal cam.

from laptop_ai.gopro_driver import GoProDriver
import asyncio
import subprocess


class CameraExecutor:
    def __init__(self):
        self.gopro = GoProDriver()

    async def apply_camera_plan(self, plan: dict):
        cam = plan.get("camera")

        if cam == "gopro":
            await self._apply_gopro_plan(plan)
        else:
            await self._apply_internal_plan(plan)

    # -----------------------------------------------------
    # GOPRO
    # -----------------------------------------------------
    async def _apply_gopro_plan(self, plan):
        fps = plan.get("fps", 60)
        shutter = plan.get("shutter")
        iso = plan.get("iso_limit")
        ev = plan.get("ev")
        color = plan.get("color")

        # Simplified mapping — expand later
        fps_code = "8" if fps == 120 else "5"  # example code mapping

        await self.gopro.set_fps(fps_code)
        await asyncio.sleep(0.05)

        if shutter == "fast":
            await self.gopro.set_hero_shutter("2")  # example fast shutter code

        await self.gopro.set_iso_limit("3")  # iso 1600→code 3
        await self.gopro.set_ev("4")         # EV -0.5 → code 4
        await self.gopro.set_color_profile("2" if color == "flat" else "0")

    # -----------------------------------------------------
    # INTERNAL CAMERA SETTING (libcamera)
    # -----------------------------------------------------
    async def _apply_internal_plan(self, plan):
        """
        Controls RPi camera via libcamera-still / libcamera-vid.
        Only applies exposure-like parameters.
        """

        if plan.get("shutter") == "fast":
            shutter_speed = "8000"   # 1/8000s
        else:
            shutter_speed = "auto"

        iso = plan.get("iso_limit", "800")

        cmd = [
            "libcamera-vid",
            "--shutter", shutter_speed,
            "--gain", iso,
            "-t", "0"
        ]

        try:
            subprocess.Popen(cmd)
        except Exception as e:
            print("Could not apply internal camera plan:", e)
