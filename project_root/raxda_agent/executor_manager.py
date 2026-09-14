# executor_manager.py

import time
from mavlink_executor import MAVLinkExecutor
from failsafe_monitor import FailsafeMonitor

class ExecutorManager:
    """
    Connects AI → MAVLinkExecutor → Drone (via setpoints)
    """

    def __init__(self):
        self.exe = MAVLinkExecutor()
        self.safe = FailsafeMonitor()

    def execute_motion(self, motion):
        """
        motion = {
            "vx": ...,
            "vy": ...,
            "vz": ...,
            "yaw_rate": ...,
            "duration": ...
        }
        """

        duration = motion.get("duration", 0.1)
        vx = motion["vx"]
        vy = motion["vy"]
        vz = motion["vz"]
        yaw_rate = motion.get("yaw_rate", 0)

        start = time.time()

        while time.time() - start < duration:
            gps_sats = 12               # your telemetry source here
            battery = 70                # your telemetry source here
            rc_override = False         # detect from RC input channel
            obstacles_clear = True      # read from sensors

            if not self.safe.is_safe(gps_sats, battery, rc_override, obstacles_clear):
                print("[EXECUTOR] Unsafe → sending hover (vx=vy=vz=0)")
                self.exe.send_velocity_setpoint(0, 0, 0, 0)
                return

            self.exe.send_velocity_setpoint(vx, vy, vz, yaw_rate)
            time.sleep(1 / 30)
