# radxa_exec/deadman.py

class DeadMan:
    """
    Forces safe neutral output on failure.
    """

    @staticmethod
    def neutral():
        return {
            "pan": 0.0,
            "tilt": 0.0,
            "forward": 0.0,
        }