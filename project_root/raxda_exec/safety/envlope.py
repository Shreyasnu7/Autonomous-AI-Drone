class SafetyEnvelope:
    """
    Enforces motion and camera limits.
    """

    def enforce(
        self,
        command,
    ):
        command.pan = max(min(command.pan, 0.8), -0.8)
        command.tilt = max(min(command.tilt, 0.8), -0.8)
        command.forward_motion = max(
            min(command.forward_motion, 0.6), 0.0
        )
        return command