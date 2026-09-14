# radxa_exec/intent_receiver.py

class IntentReceiver:
    """
    Receives execution intent and applies safety logic.
    """

    def __init__(self, safety, gimbal, motion):
        self.safety = safety
        self.gimbal = gimbal
        self.motion = motion

    def apply(self, intent):
        if not self.safety.allow(intent):
            return

        self.gimbal.set_pan_tilt(intent.pan, intent.tilt)
        self.motion.set_forward(intent.forward_motion)