# radxa_agent/execution_bridge.py

from dataclasses import dataclass


@dataclass
class ExecutionIntent:
    pan: float
    tilt: float
    forward_motion: float
    confidence: float | None
    intent_tag: str | None


class ExecutionBridge:
    """
    Bridges high-level camera intent to execution-safe intent.
    """

    def __init__(self, exec_client):
        self.exec_client = exec_client

    def push_camera_intent(
        self,
        *,
        pan: float,
        tilt: float,
        framing_x: float,
        framing_y: float,
        forward_motion: float,
        lens_compression: float,
        confidence=None,
        intent_tag=None,
    ):
        # 🔒 Clamp and sanitize
        pan = max(-1.0, min(1.0, pan))
        tilt = max(-1.0, min(1.0, tilt))
        forward_motion = max(0.0, min(1.0, forward_motion))

        intent = ExecutionIntent(
            pan=pan,
            tilt=tilt,
            forward_motion=forward_motion,
            confidence=confidence,
            intent_tag=intent_tag,
        )

        self.exec_client.send(intent)