from dataclasses import dataclass


@dataclass
class CameraCommand:
    pan: float               # -1.0 → 1.0
    tilt: float              # -1.0 → 1.0
    forward_motion: float    # abstract motion scalar
    framing_x: float         # -1.0 → 1.0
    framing_y: float         # -1.0 → 1.0
    lens_compression: float  # 0.3 → 1.0