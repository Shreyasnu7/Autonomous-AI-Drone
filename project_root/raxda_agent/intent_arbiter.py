# radxa_agent/intent_arbiter.py

class IntentArbiter:
    """
    Chooses between AI intent andTnd manual intent.

    Rules:
    - Manual ALWAYS overrides AI if user moves joystick
    - AI only runs when enabled
    """

    def blend(self, ai_intent, manual_intent=None):
        # Manual override always wins
        if manual_intent is not None:
            return manual_intent

        # Otherwise AI (if exists)
        return ai_intent