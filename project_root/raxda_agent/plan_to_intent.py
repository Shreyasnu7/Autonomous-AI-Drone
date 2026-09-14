# radxa_agent/plan_to_intent.py

from dataclasses import dataclass
from typing import Dict, Any


@dataclass
class ExecutionIntent:
    """
    Minimal, execution-safe intent.
    Expanded later by camera_brain.
    """
    pan: float = 0.0
    tilt: float = 0.0
    forward_motion: float = 0.0

    motion_style: str = "neutral"
    motion_energy: float = 0.2
    lens_compression: float = 0.3
    camera_smoothness: float = 0.8


class PlanToIntentAdapter:
    """
    Converts high-level server plans into execution intents.
    No AI reasoning here.
    No safety here.
    """

    def convert(self, plan: Dict[str, Any]) -> ExecutionIntent:
        action = plan.get("action", "").lower()
        style = plan.get("style", "neutral").lower()

        intent = ExecutionIntent()

        # --- Action semantics ---
        if action == "follow":
            intent.forward_motion = 0.4
            intent.motion_style = "tracking"
        elif action == "orbit":
            intent.pan = 0.3
            intent.motion_style = "orbit"
        elif action == "hover":
            intent.forward_motion = 0.0
            intent.motion_style = "static"
        else:
            intent.motion_style = "neutral"

        # --- Style semantics ---
        if style == "cinematic":
            intent.motion_energy = 0.25
            intent.camera_smoothness = 0.9
            intent.lens_compression = 0.6
        elif style == "aggressive":
            intent.motion_energy = 0.6
            intent.camera_smoothness = 0.4
            intent.lens_compression = 0.2
        elif style == "documentary":
            intent.motion_energy = 0.2
            intent.camera_smoothness = 0.85
            intent.lens_compression = 0.4

        return intent