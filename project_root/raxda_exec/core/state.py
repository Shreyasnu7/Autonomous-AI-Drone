from dataclasses import dataclass


@dataclass
class ExecutionState:
    last_pan: float = 0.0
    last_tilt: float = 0.0
    last_forward_motion: float = 0.0

    ai_enabled: bool = False
    failsafe_active: bool = False