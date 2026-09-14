class IntentForwarder:
    def __init__(self, receiver, manual_override=None):
        self.receiver = receiver
        self.manual_override = manual_override

    def forward(self, intent):
        if self.manual_override and self.manual_override.check():
            print("🛑 Manual override active. Intent blocked.")
            return
        self.receiver.receive(intent)