# radxa_agent/manual_override.py

class ManualOverride:
    def __init__(self, safety):
        self.safety = safety
        self.active = False

    def enable(self):
        self.active = True
        self.safety.force_hold("manual_override")

    def disable(self):
        self.active = False

    def check(self):
        return self.active