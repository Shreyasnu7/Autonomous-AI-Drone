class Failsafe:
    def trigger(self, state):
        state.ai_enabled = False
        state.failsafe_active = True