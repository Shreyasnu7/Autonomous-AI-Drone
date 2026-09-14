class CameraBridge:
    """
    Interfaces with CSI camera (PiCam / similar).
    """

    def apply_framing(
        self,
        framing_x: float,
        framing_y: float,
    ):
        # Maps framing offsets to digital crop / ISP
        pass

    def apply_lens_compression(
        self,
        compression: float,
    ):
        # Maps to focal simulation / ISP params
        pass