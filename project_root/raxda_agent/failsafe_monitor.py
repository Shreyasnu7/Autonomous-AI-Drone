# failsafe_monitor.py
import time

class FailsafeMonitor:
    """
    Ensures *safe drone behavior*.
    Does NOT send dangerous commands.
    """

    def __init__(self):
        self.last_ai_msg = time.time()
        self.min_gps = 6
        self.min_batt = 20  # %
        self.is_rc_override = False

    def update_ai_timestamp(self):
        self.last_ai_msg = time.time()

    def is_safe(self, gps_sats, battery, rc_override, obstacles_clear):
        """
        True → AI command allowed.
        False → Executor must enter safe hover.
        """

        if rc_override:
            print("[FAILSAFE] RC override detected → Human control")
            return False

        if gps_sats < self.min_gps:
            print("[FAILSAFE] GPS too low")
            return False

        if battery < self.min_batt:
            print("[FAILSAFE] Battery critical")
            return False

        if not obstacles_clear:
            print("[FAILSAFE] Obstacle block")
            return False

        if time.time() - self.last_ai_msg > 1.5:
            print("[FAILSAFE] AI timeout → hover")
            return False

        return True
