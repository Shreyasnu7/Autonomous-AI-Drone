import asyncio
import logging
import os
from bleak import BleakClient

logger = logging.getLogger('RadxaGoProProxy')

# MA-25 FIX: GoPro MAC configurable via env var (update per device after pairing)
GOPRO_MAC = os.environ.get("GOPRO_MAC", "E1:9D:9A:BB:E5:5B")

# GoPro BLE Command & Settings UUIDs
CMD_UUID    = "b5f90072-aa8d-11e3-9046-0002a5d5c51b"  # Command channel
SETTING_UUID = "b5f90074-aa8d-11e3-9046-0002a5d5c51b"  # Settings channel

# --- GoPro Raw BLE Setting IDs ---
# These are the raw integer IDs from the GoPro BLE API spec
SETTING_ID = {
    "resolution":            2,
    "fps":                   3,
    "shutter":               73,
    "iso_min":               102,
    "iso_max":               13,
    "white_balance":         11,
    "color_profile":         19,
    "sharpness":             12,
    "hypersmooth":           135,
    "ev_comp":               15,   # exposure_compensation
}

# Value mappings for string-based AI settings
COLOR_MAP = {
    "flat": 2, "vibrant": 1, "natural": 0, "log": 3, "gp-log": 3
}
WB_MAP = {
    "auto": 0, "2300k": 1, "2800k": 2, "3200k": 3, "4000k": 4,
    "4800k": 5, "5500k": 6, "6000k": 7, "6500k": 8, "native": 9
}
SHARPNESS_MAP = {
    "low": 2, "medium": 1, "high": 0
}
HYPERSMOOTH_MAP = {
    "off": 0, "low": 1, "auto": 3, "high": 2, "boost": 4
}


class RadxaGoProProxy:
    """
    Runs LOCALLY on the Drone (Radxa Zero 3W).
    Uses direct BLE via bleak — no sudo, no slow open-gopro wrapper.
    Connects in under 2 seconds. Stays connected for the full flight.
    """
    def __init__(self):
        self.client = None
        self.connected = False

    async def connect(self):
        try:
            logger.info(f"Radxa connecting to GoPro Hero 12 [{GOPRO_MAC}] via BLE...")
            self.client = BleakClient(GOPRO_MAC)
            await self.client.connect()
            self.connected = self.client.is_connected
            if self.connected:
                logger.info("Radxa ❤️ GoPro: BLE Link ACTIVE (Fast Direct Mode)")
            return self.connected
        except Exception as e:
            logger.error(f"GoPro BLE Connect Failed: {e}")
            return False

    async def disconnect(self):
        if self.client and self.connected:
            await self.client.disconnect()
            self.connected = False

    def _encode_setting(self, setting_id: int, value: int) -> bytearray:
        """Encodes a GoPro BLE TLV setting packet."""
        # Format: [len_of_rest, setting_id, value_len, value_byte(s)]
        return bytearray([3, setting_id, 1, value & 0xFF])

    async def _send_setting(self, setting_id: int, value: int):
        """Writes a single raw setting to the GoPro over BLE."""
        if not self.connected or not self.client:
            return
        try:
            packet = self._encode_setting(setting_id, value)
            await self.client.write_gatt_char(SETTING_UUID, packet)
        except Exception as e:
            logger.error(f"BLE Setting Send Error (id={setting_id}): {e}")

    async def _shutter(self, on: bool):
        """Starts or stops recording."""
        cmd = bytearray([0x03, 0x01, 0x01, 0x01 if on else 0x00])
        try:
            await self.client.write_gatt_char(CMD_UUID, cmd)
        except Exception as e:
            logger.error(f"BLE Shutter Error: {e}")

    async def start_preview_stream(self):
        """
        Enable the GoPro's WiFi preview stream via BLE.

        GoPro Hero 12 BLE Protocol:
        - Command 0x03, 0x01, 0x01, 0x01 = Shutter Start
        - Command 0x02, 0x01, 0x00 = Set Turbo Transfer Off
        - Command for WiFi AP mode enable = needed for stream

        The GoPro exposes its preview stream at:
        - UDP: udp://@:8554 (MPEG-TS, port 8554 on the GoPro's WiFi network)
        - The Radxa must be connected to the GoPro's WiFi AP first

        Steps:
        1. Enable WiFi AP on GoPro via BLE
        2. Radxa connects to GoPro WiFi (SSID from BLE or stored)
        3. Open UDP stream at udp://@0.0.0.0:8554

        GoPro BLE WiFi commands:
        - GP-0091 (WiFi AP SSID) and GP-0092 (WiFi AP Password) are readable
        - Command to enable WiFi: write [0x03, 0x17, 0x01, 0x01] to CMD_UUID
        - Command to start preview: write [0x04, 0x02, 0x01, 0x02, 0x00] to CMD_UUID
        """
        if not self.connected or not self.client:
            logger.warning("Cannot start stream: BLE not connected")
            return False

        try:
            # Step 1: Enable WiFi AP on GoPro
            # BLE Command: Set WiFi AP On
            wifi_on_cmd = bytearray([0x03, 0x17, 0x01, 0x01])
            await self.client.write_gatt_char(CMD_UUID, wifi_on_cmd)
            logger.info("GoPro WiFi AP: Enabling...")

            import asyncio
            await asyncio.sleep(3)  # Wait for WiFi to come up

            # Step 2: Start preview stream
            # BLE Command: Start Preview Stream (Low-latency UDP)
            # Format: [len, cmd_id=0x02, subcmd, params...]
            # 0x02 = Preset/Mode control
            # For preview: some GoPros use the webcam mode or live preview
            preview_cmd = bytearray([0x03, 0x02, 0x01, 0x00])
            await self.client.write_gatt_char(CMD_UUID, preview_cmd)
            logger.info("GoPro Preview Stream: Starting...")

            await asyncio.sleep(2)

            logger.info(
                "GoPro stream should now be available at udp://@0.0.0.0:8554\n"
                "IMPORTANT: Radxa must be connected to GoPro's WiFi network!\n"
                "Use: nmcli dev wifi connect <GoPro_SSID> password <GoPro_Pass>"
            )
            return True

        except Exception as e:
            logger.error(f"Failed to start preview stream: {e}")
            return False

    async def enable_usb_webcam_mode(self):
        """
        Switch GoPro Hero 12 to USB Webcam mode via BLE.

        Once activated, the GoPro appears as a UVC webcam on the USB-C host
        (Radxa) at /dev/video*. This gives the lowest latency, most reliable
        video feed — no WiFi needed for video.

        GoPro BLE Protocol:
        - Command to enter webcam mode: Feature ID 0x72 (Webcam)
        - Webcam preset load: [0x06, 0x40, 0x04, 0x00, 0x00, 0x00, 0xF9]
          (Load preset 0xF9 = 249 = Webcam mode on Hero 12)
        - Then USB-C connection provides UVC video
        """
        if not self.connected or not self.client:
            logger.warning("Cannot enable USB webcam: BLE not connected")
            return False

        try:
            # Step 1: Load Webcam preset (preset ID 0xF9 = 249 for Hero 12)
            # BLE Set Preset command: [total_len, cmd=0x40, subcmd=0x04, ...preset_id_le32]
            preset_webcam = bytearray([0x06, 0x40, 0x04, 0xF9, 0x00, 0x00, 0x00])
            await self.client.write_gatt_char(CMD_UUID, preset_webcam)
            logger.info("GoPro: Loading USB Webcam preset (0xF9)...")

            await asyncio.sleep(3)  # Wait for mode switch

            # Step 2: Start webcam via BLE (tells GoPro to begin UVC output)
            # Webcam start command
            webcam_start = bytearray([0x03, 0x02, 0x01, 0x01])
            await self.client.write_gatt_char(CMD_UUID, webcam_start)
            logger.info("GoPro: USB Webcam mode ACTIVE — check /dev/video* on Radxa")

            await asyncio.sleep(2)
            return True

        except Exception as e:
            logger.error(f"Failed to enable USB webcam mode: {e}")
            return False

    async def stop_preview_stream(self):
        """Stop the GoPro's WiFi preview stream."""
        if not self.connected or not self.client:
            return False
        try:
            # Disable WiFi AP
            wifi_off_cmd = bytearray([0x03, 0x17, 0x01, 0x00])
            await self.client.write_gatt_char(CMD_UUID, wifi_off_cmd)
            logger.info("GoPro WiFi AP: Disabled")
            return True
        except Exception as e:
            logger.error(f"Failed to stop stream: {e}")
            return False

    async def get_wifi_credentials(self):
        """
        Read the GoPro's WiFi SSID and Password via BLE.
        These are needed to connect Radxa to GoPro's WiFi network.

        Returns: (ssid, password) or (None, None) on failure.
        """
        if not self.connected or not self.client:
            return None, None

        WIFI_SSID_UUID = "b5f90002-aa8d-11e3-9046-0002a5d5c51b"
        WIFI_PASS_UUID = "b5f90003-aa8d-11e3-9046-0002a5d5c51b"

        try:
            ssid_bytes = await self.client.read_gatt_char(WIFI_SSID_UUID)
            pass_bytes = await self.client.read_gatt_char(WIFI_PASS_UUID)
            ssid = ssid_bytes.decode('utf-8').strip('\x00')
            password = pass_bytes.decode('utf-8').strip('\x00')
            logger.info(f"GoPro WiFi: SSID={ssid}")
            return ssid, password
        except Exception as e:
            logger.error(f"Failed to read WiFi credentials: {e}")
            return None, None

    async def handle_remote_ai_command(self, payload: dict):
        """
        Takes AI JSON or App commands from the cloud and executes via BLE.
        Called by real_bridge_service when it receives a GOPRO_SETTINGS packet.

        Supports two formats:
        1. App buttons: {"action": "record_start" | "record_stop" | "photo" | "timelapse" | "set_color", "value": "..."}
        2. AI settings: {"settings": {"color_profile": "flat", "shutter": true, ...}}
        """
        if not self.connected:
            logger.warning("Dropping AI command: GoPro BLE not connected.")
            await self.connect()
            if not self.connected:
                return

        # --- Handle App Button Actions ---
        action = payload.get("action")
        if action:
            try:
                if action == "record_start":
                    await self._shutter(True)
                    logger.info("GoPro: RECORDING STARTED")
                elif action == "record_stop":
                    await self._shutter(False)
                    logger.info("GoPro: RECORDING STOPPED")
                elif action == "photo":
                    # Switch to photo mode, take photo, switch back
                    # Photo preset = 0x01000001 on Hero 12
                    photo_preset = bytearray([0x06, 0x40, 0x04, 0x01, 0x00, 0x00, 0x01])
                    await self.client.write_gatt_char(CMD_UUID, photo_preset)
                    await asyncio.sleep(1)
                    await self._shutter(True)  # Take photo
                    await asyncio.sleep(0.5)
                    # Switch back to video preset
                    video_preset = bytearray([0x06, 0x40, 0x04, 0x00, 0x00, 0x00, 0x00])
                    await self.client.write_gatt_char(CMD_UUID, video_preset)
                    logger.info("GoPro: PHOTO CAPTURED")
                elif action == "timelapse":
                    # Timelapse preset on Hero 12
                    tl_preset = bytearray([0x06, 0x40, 0x04, 0x02, 0x00, 0x00, 0x00])
                    await self.client.write_gatt_char(CMD_UUID, tl_preset)
                    await asyncio.sleep(1)
                    await self._shutter(True)
                    logger.info("GoPro: TIMELAPSE STARTED")
                elif action == "set_color":
                    color_val = payload.get("value", "natural")
                    val = COLOR_MAP.get(str(color_val).lower(), 0)
                    await self._send_setting(SETTING_ID["color_profile"], int(val))
                    logger.info(f"GoPro: COLOR PROFILE → {color_val}")
                elif action == "set_resolution":
                    res_val = int(payload.get("value", 12))
                    await self._send_setting(SETTING_ID["resolution"], res_val)
                    logger.info(f"GoPro: RESOLUTION → {res_val}")
                elif action == "set_fps":
                    fps_val = int(payload.get("value", 8))
                    await self._send_setting(SETTING_ID["fps"], fps_val)
                    logger.info(f"GoPro: FPS → {fps_val}")
                else:
                    logger.warning(f"Unknown GoPro action: {action}")
            except Exception as e:
                logger.error(f"GoPro action '{action}' failed: {e}")
            return  # Action handled, don't process settings

        # --- Handle AI Settings (bulk) ---
        settings = payload.get("settings", {})
        labs_cmd = payload.get("labs")

        for ai_feature, ai_value in settings.items():
            logger.info(f"GoPro ← AI: {ai_feature} = {ai_value}")
            try:
                if ai_feature == "shutter":
                    await self._shutter(bool(ai_value))

                elif ai_feature == "color_profile":
                    val = COLOR_MAP.get(str(ai_value).lower(), ai_value)
                    await self._send_setting(SETTING_ID["color_profile"], int(val))

                elif ai_feature == "white_balance":
                    val = WB_MAP.get(str(ai_value).lower(), ai_value)
                    await self._send_setting(SETTING_ID["white_balance"], int(val))

                elif ai_feature == "sharpness":
                    val = SHARPNESS_MAP.get(str(ai_value).lower(), ai_value)
                    await self._send_setting(SETTING_ID["sharpness"], int(val))

                elif ai_feature == "hypersmooth":
                    val = HYPERSMOOTH_MAP.get(str(ai_value).lower(), ai_value)
                    await self._send_setting(SETTING_ID["hypersmooth"], int(val))

                elif ai_feature == "exposure_compensation":
                    # EV Comp range: -2 to +2 mapped to 0-8 (GoPro internal)
                    val = int(ai_value) + 4  # Center = 4 (0 EV)
                    await self._send_setting(SETTING_ID["ev_comp"], val)

                elif ai_feature in SETTING_ID:
                    await self._send_setting(SETTING_ID[ai_feature], int(ai_value))

            except Exception as e:
                logger.error(f"Error applying {ai_feature}: {e}")

        if labs_cmd:
            logger.info(f"GoPro Labs command (not yet implemented via BLE): {labs_cmd}")
