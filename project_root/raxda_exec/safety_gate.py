# radxa_exec/safety_gate.py

import time


class SafetyLimits:
    """
    Placeholder for future limits:
    - max tilt
    - max speed
    - min battery
    - geofence
    """
    def __init__(self):
        self.max_tilt = 30.0
        self.max_forward = 1.0


class SafetyGate:
    """
    SINGLE SOURCE OF TRUTH.
    If this gate says NO — nothing moves.
    """

    def __init__(self, limits: SafetyLimits):
        self.limits = limits

        self._holds = set()
        self._last_heartbeat = time.time()
        self._heartbeat_timeout = 2.0  # seconds

    # ---------------- HEARTBEAT ----------------

    def tick(self):
        """Called by Heartbeat before execution"""
        self._last_heartbeat = time.time()

    def heartbeat_ok(self) -> bool:
        return (time.time() - self._last_heartbeat) <= self._heartbeat_timeout

    # ---------------- HOLDS ----------------

    def force_hold(self, reason: str):
        if reason not in self._holds:
            print(f"🛑 SAFETY HOLD: {reason}")
            self._holds.add(reason)

    def release_hold(self, reason: str):
        if reason in self._holds:
            print(f"✅ SAFETY HOLD RELEASED: {reason}")
            self._holds.remove(reason)

    # ---------------- EXECUTION GATE ----------------

    def allow(self) -> bool:
        """
        Called by Executor BEFORE any motion.
        This is FINAL authority.
        """
        if not self.heartbeat_ok():
            self.force_hold("heartbeat_timeout")
            return False

        return len(self._holds) == 0

    # ---------------- CLAMP ----------------

    def clamp(self, pan, tilt, forward):
        tilt = max(-self.limits.max_tilt, min(self.limits.max_tilt, tilt))
        forward = max(-self.limits.max_forward, min(self.limits.max_forward, forward))
        return pan, tilt, forward