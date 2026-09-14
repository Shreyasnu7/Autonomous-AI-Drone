# radxa_exec/rate_limiter.py

class RateLimiter:
    """
    Limits per-step change to avoid sudden motion.
    """

    def __init__(self, max_delta: float):
        self.max_delta = float(max_delta)
        self.prev = 0.0

    def step(self, value: float) -> float:
        delta = value - self.prev
        if delta > self.max_delta:
            value = self.prev + self.max_delta
        elif delta < -self.max_delta:
            value = self.prev - self.max_delta
        self.prev = value
        return value