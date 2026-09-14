# File: radxa_agent/plan_receiver.py

class PlanReceiver:
    """
    Receives safe plans from server.
    This file does NOT execute flight movement.
    """

    def __init__(self):
        self.latest_plan = None

    async def handle(self, plan_packet):
        self.latest_plan = plan_packet
        print("[PlanReceiver] safe plan received")
        print(plan_packet)
