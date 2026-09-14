# radxa_exec/executor.py

from radxa_exec.safety_gate import SafetyGate
from radxa_exec.rate_limiter import RateLimiter
from radxa_exec.deadman import DeadMan
from radxa_exec.actuators import Gimbal, Motion


class Executor:
    """
    The ONLY place where intent becomes motion.
    """

    def __init__(self, safety: SafetyGate, gimbal: Gimbal, motion: Motion):
        self.safety = safety
        self.gimbal = gimbal
        self.motion = motion

        self.pan_rl = RateLimiter(max_delta=0.05)
        self.tilt_rl = RateLimiter(max_delta=0.05)
        self.fwd_rl = RateLimiter(max_delta=0.05)

    def apply(self, intent):
        if not self.safety.allow():
            n = DeadMan.neutral()
            self.gimbal.set(n["pan"], n["tilt"])
            self.motion.set_forward(n["forward"])
            return

        pan, tilt, fwd = self.safety.clamp(
            intent.pan, intent.tilt, intent.forward_motion
        )

        pan = self.pan_rl.step(pan)
        tilt = self.tilt_rl.step(tilt)
        fwd = self.fwd_rl.step(fwd)

        # Check heartbeat before applying
        if not self.safety.heartbeat_ok():
            print("Heartbeat lost. Execution blocked.")
            n = DeadMan.neutral()
            self.gimbal.set(n["pan"], n["tilt"])
            self.motion.set_forward(n["forward"])
            return

        self.gimbal.set(pan, tilt)
        self.motion.set_forward(fwd)