import numpy as np

class SafetyEnvelope:
    """
    Predicts collision & dynamic safety risks for the next 2–4 seconds
    before a curve is accepted.
    """

    def __init__(self, horizon=3.0, samples=40, min_clearance=1.2):
        self.horizon = float(horizon)
        self.samples = int(samples)
        self.min_clearance = float(min_clearance)

    # -------------------------------------------------------------

    def predict_collision(self, curve, obstacles):
        """
        Returns True if the curve is unsafe.
        """
        if curve is None:
            return True

        ts = np.linspace(0, 1, self.samples)
        for t in ts:
            p = curve.point(t)

            for o in obstacles:
                pos = np.array(o["pos"], dtype=float)
                r = float(o["radius"])

                if np.linalg.norm(p - pos) < (r + self.min_clearance):
                    return True

        return False

    # -------------------------------------------------------------

    def check_dynamics(self, curve, max_speed=8.0, max_accel=6.0):
        """
        Simple motion-limit check: if tangent or curvature is too extreme,
        mark curve unsafe.
        """
        ts = np.linspace(0, 1, self.samples)
        last_v = None

        for t in ts:
            v = curve.tangent(t)

            speed = np.linalg.norm(v)
            if speed > max_speed:
                return True

            if last_v is not None:
                accel = np.linalg.norm(v - last_v)
                if accel > max_accel:
                    return True

            last_v = v

        return False

    # -------------------------------------------------------------

    def evaluate(self, curve, obstacles):
        """
        Returns:
            safe=True/False
        """

        if self.predict_collision(curve, obstacles):
            return False

        if self.check_dynamics(curve):
            return False

        return True

    def is_critical(self, environment_state):
        """
        Reflexive check: is there an obstacle IMMEDIATELY in front (< 1.5m)?
        Returns True if emergency stop is needed.
        """
        if not environment_state or "obstacles" not in environment_state:
            return False

        for obs in environment_state["obstacles"]:
            dist = obs.get("distance", 99.0)
            if dist < 1.5:  # Critical distance
                return True
        return False
