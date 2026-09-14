# radxa_agent/camera_command_receiver.py

import json
from radxa_agent.execution_bridge import ExecutionBridge


class CameraCommandReceiver:
    """
    Receives CameraCommand from Camera Brain.
    """

    def __init__(self, exec_bridge: ExecutionBridge):
        self.exec_bridge = exec_bridge

    def handle_packet(self, raw: bytes):
        msg = json.loads(raw.decode())

        if msg.get("type") != "camera_command":
            return

        data = msg["data"]

        self.exec_bridge.push_camera_intent(
            pan=data["pan"],
            tilt=data["tilt"],
            framing_x=data["framing_x"],
            framing_y=data["framing_y"],
            forward_motion=data["forward_motion"],
            lens_compression=data["lens_compression"],
            confidence=data.get("confidence"),
            intent_tag=data.get("intent_tag"),
        )