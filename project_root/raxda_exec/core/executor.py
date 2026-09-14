from .state import ExecutionState
from .rate_limiter import rate_limit
from ..safety.envelope import SafetyEnvelope
from ..gimbal.gimbal_bridge import GimbalBridge
from ..camera.camera_bridge import CameraBridge


class RadxaExecutor:
    """
    Executes camera commands safely.
    """

    def __init__(self):
        self.state = ExecutionState()
        self.safety = SafetyEnvelope()
        self.gimbal = GimbalBridge()
        self.camera = CameraBridge()

    def execute(self, command):
        if not self.state.ai_enabled:
            return

        command = self.safety.enforce(command)

        pan = rate_limit(command.pan, self.state.last_pan, 0.05)
        tilt = rate_limit(command.tilt, self.state.last_tilt, 0.05)

        self.gimbal.send(pan, tilt)
        self.camera.apply_framing(
            command.framing_x,
            command.framing_y,
        )
        self.camera.apply_lens_compression(
            command.lens_compression,
        )

        self.state.last_pan = pan
        self.state.last_tilt = tilt