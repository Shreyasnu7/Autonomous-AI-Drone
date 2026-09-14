# File: radxa_agent/curve_streamer.py

import asyncio
import time
import numpy as np
from radxa_agent.safe_executor import SafeExecutor


class CurveStreamer:
    """
    Streams Bezier curve points at 50–100 Hz to the drone
    using SafeExecutor + MAVLink setpoints.
    """

    def __init__(self, rate_hz=80):
        self.executor = SafeExecutor()
        self.rate_hz = rate_hz
        self.dt = 1.0 / rate_hz

        self.curve = None
        self.start_time = None
        self.duration = 4.0    # default cinematic curve length

        self.streaming = False

    async def start(self):
        """Connect MAVLink."""
        await self.executor.start()

    def load_curve(self, bezier_curve, duration_sec=4.0):
        """
        Provide a BezierCurve instance.
        """
        self.curve = bezier_curve
        self.duration = duration_sec
        self.start_time = time.time()

    async def run(self):
        """
        Main 50–100 Hz loop — call this in a task.
        """
        print(f"[CurveStreamer] Running at {self.rate_hz} Hz")

        self.streaming = True
        while True:
            if self.curve is None:
                await asyncio.sleep(self.dt)
                continue

            # progress [0 → 1]
            elapsed = time.time() - self.start_time
            t = max(0.0, min(1.0, elapsed / self.duration))

            # Evaluate Bezier curve
            pos = self.curve.point(t)  # numpy array [x,y,z]

            primitive = {
                "action": "MOVE_TO",
                "params": {
                    "x": float(pos[0]),
                    "y": float(pos[1]),
                    "z": float(pos[2]),
                    "yaw": 0  # yaw could be controlled separately
                }
            }

            # send point to SafeExecutor → MAVLink → FC
            await self.executor.execute_primitive(primitive)

            # stop when finished
            if t >= 1.0:
                print("[CurveStreamer] Curve complete")
                self.curve = None

            await asyncio.sleep(self.dt)
