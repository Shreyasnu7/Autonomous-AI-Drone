import time

class Heartbeat:
    def __init__(self, safety, timeout=2.0):
        self.safety = safety
        self.timeout = timeout
        self.last_tick = time.time()

    def tick(self):
        self.last_tick = time.time()

    def check(self):
        if time.time() - self.last_tick > self.timeout:
            self.safety.force_hold("heartbeat_timeout")
            return False
        return True