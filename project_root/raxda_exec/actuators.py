# radxa_exec/actuators.py

class Gimbal:
    def set(self, pan: float, tilt: float):
        # Abstract actuator interface
        pass


class Motion:
    def set_forward(self, value: float):
        # Abstract actuator interface
        pass