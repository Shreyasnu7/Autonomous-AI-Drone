# mavlink_executor.py

import time
from pymavlink import mavutil
from threading import Lock

from config.mavlink_config import (
    MAVLINK_CONNECTION,
    SYSTEM_ID, COMPONENT_ID,
    SETPOINT_RATE_HZ,
)

class MAVLinkExecutor:
    """
    Sends raw MAVLink SET_POSITION_TARGET_LOCAL_NED messages.
    FC handles all stabilization — safe for minors.
    """

    def __init__(self):
        self.master = mavutil.mavlink_connection(MAVLINK_CONNECTION)
        self.lock = Lock()
        self.last_cmd_time = time.time()

        print("[MAVLINK] Waiting for heartbeat...")
        self.master.wait_heartbeat()
        print("[MAVLINK] Heartbeat received.")

        self._set_guided_mode()

    def _set_guided_mode(self):
        """Tell autopilot to enter GUIDED / OFFBOARD."""
        try:
            self.master.set_mode_apm("GUIDED")
        except Exception:
            pass

        print("[MAVLINK] Mode set → GUIDED/OFFBOARD ready")

    def send_velocity_setpoint(self, vx, vy, vz, yaw_rate=0.0):
        """
        AI planner calls this function with desired local velocities.
        """

        with self.lock:
            msg = self.master.mav.set_position_target_local_ned_encode(
                0,
                SYSTEM_ID,
                COMPONENT_ID,
                mavutil.mavlink.MAV_FRAME_BODY_NED,   # body frame = lowest latency
                0b0000111111000111,                   # ignore pos/accel, use velocity+yaw
                0, 0, 0,                               # position ignored
                float(vx),
                float(vy),
                float(vz),
                0, 0, 0,                               # accel ignored
                float(yaw_rate)
            )
            self.master.mav.send(msg)
            self.last_cmd_time = time.time()

    def heartbeat(self):
        """Send an offboard heartbeat to keep FC trusting us."""
        with self.lock:
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
