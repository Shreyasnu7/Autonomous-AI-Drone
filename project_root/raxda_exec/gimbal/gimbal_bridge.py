class GimbalBridge:
    """
    Sends pan/tilt commands to ESP32 or direct PWM.
    """

    def send(
        self,
        pan: float,
        tilt: float,
    ):
        # UART / GPIO / I2C
        pass