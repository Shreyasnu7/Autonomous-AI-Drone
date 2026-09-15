import asyncio
import cv2
import json
import socket
import time
import os
import aiohttp
from aiohttp import web  # module-level so request handlers (handle_snapshot/handle_stream) can use `web`
import websockets
from pymavlink import mavutil

# MA-29 FIX: Import onboard obstacle avoidance (potential field method)
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'raxda'))
try:
    from ai.avoidance import ObstacleAvoidance
    AVOIDANCE_AVAILABLE = True
    print("✅ Smart obstacle avoidance loaded (potential field)")
except ImportError:
    AVOIDANCE_AVAILABLE = False
    print("⚠️ Smart avoidance not available, using basic brake only")
# --- CONFIG ---
# Cloud server (Render) — used when Tailscale direct connection not available
SERVER_URL = os.environ.get("SERVER_URL", "wss://drone-server-r0qe.onrender.com/ws/connect/RADXA_X")
API_URL = os.environ.get("API_URL", "https://drone-server-r0qe.onrender.com")
# Tailscale IPs for direct streaming (faster than cloud relay).
# Defaults are PLACEHOLDERS -- override via TS_LAPTOP_IP / TS_PHONE_IP.
TAILSCALE_LAPTOP_IP = os.environ.get("TS_LAPTOP_IP", "100.64.0.10")
TAILSCALE_PHONE_IP = os.environ.get("TS_PHONE_IP", "100.64.0.20")
# Auto-detect FC UART port based on board
# Cubie A7Z (Allwinner A733): /dev/ttyAS0 (Pin 8/10)
# Radxa Zero 3W (RK3566): /dev/ttyS2 (Pin 8/10)
import os
if os.path.exists("/dev/ttyAS0"):
    FC_PORT = os.environ.get("FC_PORT", "/dev/ttyAS0")
elif os.path.exists("/dev/ttyS0"):
    FC_PORT = os.environ.get("FC_PORT", "/dev/ttyS0")
else:
    FC_PORT = os.environ.get("FC_PORT", "/dev/ttyS2")
FC_BAUD = 57600

# ===== BENCH SAFETY: block ALL automatic arming so propellers can NEVER spin =====
# Drone is not in flight position. The bridge must not auto-arm or stick-arm the FC.
# Set to False ONLY when the drone is set up correctly and you want to fly.
ALLOW_AUTO_ARM = os.environ.get("ALLOW_AUTO_ARM", "0") == "1"

# V60: Camera Constants
CAM_WIDTH = 1280
CAM_HEIGHT = 720

# --- TAILSCALE CONFIG ---
# Stable 100.x.x.x IPs via Tailscale mesh VPN (works across any network/4G/WiFi)
TAILSCALE_LAPTOP_IP = os.environ.get("TS_LAPTOP_IP", "100.64.0.10")
TAILSCALE_PHONE_IP = os.environ.get("TS_PHONE_IP", "100.64.0.20")  # ground-station-phone
TAILSCALE_VIDEO_PORT = 8554   # UDP video stream port
TAILSCALE_RADXA_IP = os.environ.get("TS_SELF_IP", "100.64.0.30")  # this device; placeholder default
class SafetyEnvelope:
    """
    Real-time safety gate using ALL available sensors:
    - 4x VL53L1X ToF sensors (ESP32: t1=front, t2=right, t3=back, t4=left)
    - YDLidar X2 (360° 2D scan, closest obstacle distance)
    - Altitude from barometer/GPS fusion

    Actions are classified into risk tiers:
    - EMERGENCY (always allowed): LAND, DISARM, RTL, UPDATE_CONFIG
    - MOVEMENT (blocked if obstacle in path): TAKEOFF, GUIDED, ORBIT,
      UPLOAD_MISSION, joystick with forward/lateral component
    - CAMERA-ONLY (always allowed): GIMBAL, CAPTURE, GOPRO_SETTINGS, RECORD
    """

    # Minimum safe distances (meters)
    CRITICAL_DIST = 0.5     # Emergency stop — too close to anything
    CAUTION_DIST = 1.0      # Slow down / block autonomous actions
    WARN_DIST = 2.0         # Advisory — AI should start planning avoidance

    # Actions that should never be blocked (safety-critical or harmless)
    ALWAYS_ALLOW = {
        'LAND', 'DISARM', 'RTL', 'UPDATE_CONFIG',
        'GIMBAL', 'CAPTURE', 'GOPRO_SETTINGS', 'RECORD', 'RECORD_STOP',
        'USER_GPS', 'PING',
        'RETURN_TO_USER', 'RTH_USER', 'RETURN_USER',  # Emergency return
        'SET_BATT_RTH',  # Config change, not movement
        'STABILIZE', 'ALT_HOLD', 'LOITER', 'POSHOLD',  # Mode switches (safe)
    }

    # Actions that involve drone movement
    MOVEMENT_ACTIONS = {
        'TAKEOFF', 'ORBIT', 'UPLOAD_MISSION', 'GUIDED',
        'GOTO', 'FOLLOW', 'VELOCITY', 'MOVE', 'AI_PLAN', 'CMD_VEL', 'COMMAND',
        'FOLLOW_ME', 'GUIDED', 'AUTO',
    }  # NOTE: 'ARM' removed — arming just idles motors, must not be blocked by proximity (was blocking indoor arm)

    def __init__(self, telemetry_cache):
        self.telem = telemetry_cache
        self.last_warning = 0

    def _get_min_tof_distance(self):
        """Get minimum distance from all 4 ToF sensors. Returns (min_dist, direction)."""
        sensors = {
            'front': self.telem.get('t1', -1),
            'right': self.telem.get('t2', -1),
            'back':  self.telem.get('t3', -1),
            'left':  self.telem.get('t4', -1),
        }
        # Drop the whole set once it has gone stale. Without this the gate kept approving or
        # refusing movement using clearances measured before the ESP32 link dropped -- readings
        # that describe where the aircraft USED to be.
        _age = time.time() - float(self.telem.get('_tof_t', 0) or 0)
        if self.telem.get('_tof_t') and _age > float(os.environ.get('TOF_STALE_S', '1.5')):
            sensors = {}

        # Filter out invalid readings (-1 means no sensor or no reading)
        valid = {k: v / 1000.0 for k, v in sensors.items()
                 if v > 0 and v < 8000}  # ToF max ~8m, values in mm
        if not valid:
            # No usable proximity data. This returns "clear" so the aircraft stays flyable
            # without ToF, but the operator should know the gate is running blind rather than
            # confirming open space.
            if not getattr(self, '_blind_warned', False):
                self._blind_warned = True
                print("⚠️ SAFETY GATE BLIND: no valid ToF readings "
                      "(absent or stale) - proximity checks are not protecting you")
            return 9.9, 'none'
        if getattr(self, '_blind_warned', False):
            self._blind_warned = False
            print("✓ SAFETY GATE: ToF readings live again")
        min_dir = min(valid, key=valid.get)
        return valid[min_dir], min_dir

    def _get_lidar_min_distance(self):
        """Get minimum distance from LiDAR (already in meters in telem cache)."""
        return self.telem.get('lidar_dist', 9.9)

    def _get_closest_obstacle(self):
        """Fuse ToF + LiDAR to get the absolute closest obstacle."""
        tof_dist, tof_dir = self._get_min_tof_distance()
        lidar_dist = self._get_lidar_min_distance()
        if tof_dist < lidar_dist:
            return tof_dist, f'tof_{tof_dir}'
        return lidar_dist, 'lidar'

    def validate_auto_action(self, action):
        """
        Returns True if the action is safe to execute.
        Returns False if the action should be blocked due to obstacle proximity.
        """
        action_upper = action.upper().strip() if isinstance(action, str) else ''

        # Always allow emergency and camera-only actions
        if action_upper in self.ALWAYS_ALLOW:
            return True

        # Get closest obstacle from all sensors
        closest_dist, source = self._get_closest_obstacle()

        # CRITICAL: Too close — block ALL movement
        if closest_dist < self.CRITICAL_DIST and action_upper in self.MOVEMENT_ACTIONS:
            print(f"🛑 SAFETY BLOCK [{source}]: {closest_dist:.2f}m — "
                  f"CRITICAL proximity, blocking {action_upper}")
            return False

        # CAUTION: Close — block autonomous movement, allow manual override
        if closest_dist < self.CAUTION_DIST and action_upper in self.MOVEMENT_ACTIONS:
            # Allow ARM (pilot may need to arm to land/recover)
            if action_upper == 'ARM':
                return True
            now = time.time()
            if now - self.last_warning > 2.0:
                print(f"⚠️ SAFETY CAUTION [{source}]: {closest_dist:.2f}m — "
                      f"blocking {action_upper}")
                self.last_warning = now
            return False

        # WARN: Advisory only (log but allow)
        if closest_dist < self.WARN_DIST and action_upper in self.MOVEMENT_ACTIONS:
            now = time.time()
            if now - self.last_warning > 5.0:
                print(f"📡 SAFETY WARN [{source}]: {closest_dist:.2f}m — "
                      f"obstacle nearby, allowing {action_upper}")
                self.last_warning = now

        return True

    def get_safety_status(self):
        """Returns current safety status dict for telemetry broadcast."""
        closest_dist, source = self._get_closest_obstacle()
        tof_dist, tof_dir = self._get_min_tof_distance()
        return {
            'closest_obstacle_m': round(closest_dist, 2),
            'obstacle_source': source,
            'tof_min_m': round(tof_dist, 2),
            'tof_direction': tof_dir,
            'lidar_min_m': round(self._get_lidar_min_distance(), 2),
            'level': ('CRITICAL' if closest_dist < self.CRITICAL_DIST
                      else 'CAUTION' if closest_dist < self.CAUTION_DIST
                      else 'WARN' if closest_dist < self.WARN_DIST
                      else 'CLEAR'),
        }
# V40: Smoothing Helper
def smooth(current_val, previous_val, alpha=0.15):
    if previous_val is None: return current_val
    return (previous_val * (1.0 - alpha)) + (current_val * alpha)
class RadxaBridge:
    def __init__(self):
        self.fc = None
        self.ws = None
        self.running = True
        self.telemetry_cache = {}
        self.boot_alt = None # V15: Relative Alt
        self.safety = SafetyEnvelope(self.telemetry_cache)
        self.cam_config = {'w': 1920, 'h': 1080, 'fps': 30, 'source': 'internal'} # Default 1080p (IMX219)
        self.target_caps = "video/x-raw,width=1920,height=1080,framerate=30/1"
        self.cam_needs_reset = False
        self.recording = False
        self.rec_out = None
        # V110: Dual camera state
        self.active_cam = 'internal'        # Which cam shows in app live feed
        self.recording_cam = 'internal'     # Which cam records/AI uses
        self.internal_cap = None            # Pi Camera V2 capture
        self.external_cap = None            # GoPro capture
        self.external_available = False     # Is GoPro connected?
        self.latest_internal_frame = None   # Latest Pi Cam frame
        self.latest_external_frame = None   # Latest GoPro frame
        self.batt_threshold = 20
        self.user_gps = None # (lat, lng)
        self.last_cloud_msg = time.time()
        self.low_batt_triggered = False
        self.is_armed = False # V107: Init missing state
        self.follow_me_active = False
        self.batt_rth_destination = 'launch'  # 'launch' or 'user'
        # LiDAR → FC proximity ring calibration (bench-verify on Mission Planner's Proximity radar):
        # an obstacle straight ahead must show at the TOP. If mirrored → flip LIDAR_DIR; if rotated →
        # adjust LIDAR_YAW_OFFSET_DEG. Env-overridable so no code edit is needed to calibrate.
        self.LIDAR_YAW_OFFSET_DEG = float(os.getenv('LIDAR_YAW_OFFSET_DEG', '0'))
        self.LIDAR_DIR = int(os.getenv('LIDAR_DIR', '-1'))   # LiDAR CCW → ArduPilot CW
        self.watchdog_triggered = False # V107: Init missing state
        self.smoothing_buffer = {'rc1': 1500, 'rc2': 1500, 'rc3': 1000, 'rc4': 1500} # V40: Smoothing State
        self.esp32_cmd_queue = asyncio.Queue()  # Commands to send to ESP32
        
        # P2.5: Local Recording State
        self.is_recording = False
        self.video_writer = None
        self.take_photo_flag = False
        self.record_start_time = 0
        
        # P2.6: Local AI Bridge & Video Server
        self.local_clients = set()
        self.latest_jpeg = None
        
        # P2.7: LOCAL GOPRO PROXY
        # Radxa executes Bluetooth settings on behalf of distant laptop AI
        self.gopro_proxy = None

        # P3.0: LIDAR INTEGRATION
        self.lidar = None
        self.lidar_min_dist = 9.9  # meters, updated by lidar loop
        # Initialize lidar cache — conservative defaults until lidar actually starts
        self.telemetry_cache['lidar_status'] = "STARTING"
        self.telemetry_cache['lidar_dist'] = 9.9  # Will be updated by lidar_loop

        # MA-29: Smart obstacle avoidance (potential field)
        self.obstacle_avoidance = ObstacleAvoidance() if AVOIDANCE_AVAILABLE else None

    async def init_gopro_proxy(self):
        try:
            from gopro_proxy import RadxaGoProProxy
            self.gopro_proxy = RadxaGoProProxy()
            connected = await self.gopro_proxy.connect()
            if connected:
                print("🔵 GOPRO BLE PROXY ACTIVE (Ready for AI Commands)")

                # P3.2: Try USB Webcam first (best), then WiFi stream (fallback)
                try:
                    # === TRY USB WEBCAM MODE FIRST ===
                    usb_ok = await self.gopro_proxy.enable_usb_webcam_mode()
                    if usb_ok:
                        print("🎥 GOPRO USB WEBCAM MODE: Enabled (checking /dev/video*)...")
                        # Give UVC device time to enumerate
                        import subprocess
                        await asyncio.sleep(3)
                        # Check if UVC device appeared
                        for idx in range(10):
                            if os.path.exists(f"/dev/video{idx}"):
                                try:
                                    result = subprocess.run(
                                        ['v4l2-ctl', '-d', f'/dev/video{idx}', '--info'],
                                        capture_output=True, text=True, timeout=3
                                    )
                                    if 'gopro' in result.stdout.lower() or 'webcam' in result.stdout.lower():
                                        print(f"✅ GOPRO USB WEBCAM DETECTED @ /dev/video{idx}")
                                        break
                                except Exception:
                                    pass

                    # GoPro WiFi: DISABLED at startup — switching kills cloud/Tailscale
                    # WiFi switch will be triggered by app command when user is ready
                    # BLE settings (resolution, color, shutter) work without WiFi
                    print("ℹ️ GOPRO: BLE ready for settings. Send GOPRO_WIFI_ON to enable video stream.")
                except Exception as e:
                    print(f"⚠️ GOPRO STREAM INIT: {e}")
        except Exception as e:
            print(f"⚠️ GOPRO PROXY INIT FAILED: {e}. Is bleak installed?")

    # MA-12 FIX: Removed dead init_lidar() and update_lidar_telemetry() methods
    # They were never called — lidar_loop() at line ~1714 handles all LiDAR via SDK

    async def execute_ai_plan(self, payload):
        """
        The AI has full authority. It sends whatever params it wants.
        We read the numbers and forward them to the flight controller.
        No action filtering. No hardcoded action lists. The AI thinks freely.

        The AI puts numeric intent into params. We support two modes:
        - Velocity mode: params has vx, vy, vz (m/s in NED frame)
        - Position mode: params has lat, lng, alt (GPS coords)
        Either mode can include yaw (degrees) or yaw_rate (deg/s).
        Gimbal values (pitch, pan) go to ESP32.
        Mode changes (mode_id or mode_name) go to FC.

        If the AI sends something we don't recognize as numbers,
        we just log it — we never crash, never block.
        """
        import math
        if not self.fc:
            print("AI PLAN: No FC connection")
            return

        p = payload.get('params', payload) if isinstance(payload, dict) else {}
        action_name = payload.get('action', 'AI') if isinstance(payload, dict) else 'AI'
        print(f"AI PLAN: {action_name} | {p}")

        # ABSOLUTE VELOCITY SANITY BOUND — the last line of defence.
        # The laptop pilot clamps its own stream to the clearance table, but a DISCRETE plan
        # (e.g. a malformed Gemini response) arrives here directly and bypasses those clamps.
        # The RC conversion below only clips the resulting PWM to [1100,1900], so an absurd
        # value such as vx=99 did not get rejected -- it became FULL STICK, i.e. maximum tilt.
        # Clamp the intent itself, and say so, rather than silently flying it.
        if isinstance(p, dict):
            _vmax = float(os.environ.get('MAX_AI_SPEED_MS', '2.0'))
            _yawmax = float(os.environ.get('MAX_AI_YAW_DPS', '90.0'))
            for _k, _lim in (('vx', _vmax), ('vy', _vmax), ('vz', _vmax), ('yaw_rate', _yawmax)):
                if _k in p:
                    try:
                        _v = float(p[_k])
                    except (TypeError, ValueError):
                        print(f"⚠️ AI PLAN: {_k}={p[_k]!r} is not a number -> dropped")
                        p[_k] = 0.0
                        continue
                    if _v != _v or _v in (float('inf'), float('-inf')):      # NaN / inf
                        print(f"⚠️ AI PLAN: {_k}={_v} not finite -> dropped")
                        p[_k] = 0.0
                    elif abs(_v) > _lim:
                        print(f"⚠️ AI PLAN: {_k}={_v} exceeds {_lim} -> clamped")
                        p[_k] = math.copysign(_lim, _v)

        try:
            # --- VELOCITY COMMAND (vx, vy, vz present) ---
            has_velocity = any(k in p for k in ('vx', 'vy', 'vz'))
            # --- POSITION COMMAND (lat, lng present) ---
            has_gps = 'lat' in p and 'lng' in p
            # --- LOCAL POSITION (x, y, z in meters from home) ---
            has_local_pos = any(k in p for k in ('x', 'y', 'z')) and not has_velocity

            # === INDOOR / NO-GPS FLIGHT: velocity -> RC override in ALT_HOLD ===
            # RESTORED (wiped 6-30..7-02; original ssh patch 6-24 — this is what made the indoor
            # flight work). ArduPilot ignores GUIDED velocity setpoints without a position estimate.
            # With no 3D GPS fix we steer attitude directly; baro holds altitude. NOTE: once VISO
            # (VISION_POSITION_ESTIMATE + EK3_SRC=ExternalNav) is configured, the FC HAS an indoor
            # position and gps_fix stays <3 only until the EKF accepts it — this path remains the
            # no-position fallback.
            # Do not act on a flight controller that has gone silent: mode, armed state and
            # gps_fix would all be read from frozen values.
            _fc_age = time.time() - getattr(self, '_fc_msg_t', 0.0)
            if getattr(self, '_fc_msg_t', 0.0) and _fc_age > float(os.environ.get('FC_STALE_S', '3.0')):
                if not getattr(self, '_fc_link_lost', False):
                    self._fc_link_lost = True
                    print(f"⚠️ FC SILENT for {_fc_age:.1f}s - ignoring AI commands "
                          f"(telemetry is frozen; the FC's own failsafes still apply)")
                return

            _has_vel_kick = any(k in p for k in ('vx', 'vy', 'vz', 'yaw_rate'))
            _gps_fix = int(self.telemetry_cache.get('gps_fix', 0) or 0)
            if _has_vel_kick and _gps_fix < 3:
                vx = float(p.get('vx', 0)); vy = float(p.get('vy', 0))
                vz = float(p.get('vz', 0)); yr = float(p.get('yaw_rate', 0))
                if self.obstacle_avoidance:
                    _tof = {k: self.telemetry_cache.get(k, -1) for k in ('t1', 't2', 't3', 't4')}
                    vx, vy, vz = self.obstacle_avoidance.adjust_velocity_command(
                        vx, vy, vz, _tof, drone_yaw_rad=self.telemetry_cache.get('yaw', 0))
                if str(self.telemetry_cache.get('mode_id', '')) not in ('ALT_HOLD', '2'):
                    self.fc.mav.set_mode_send(self.fc.target_system,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 2)  # ALT_HOLD
                _G = 250.0
                _clip = lambda v: int(max(1100, min(1900, 1500 + v)))
                rc1 = _clip( vy * _G); rc2 = _clip(-vx * _G)
                rc3 = _clip(-vz * _G); rc4 = _clip( yr * _G)
                self.fc.mav.rc_channels_override_send(
                    self.fc.target_system, self.fc.target_component,
                    rc1, rc2, rc3, rc4, 0, 0, 0, 0)
                print(f"NO-GPS AI FLIGHT (ALT_HOLD RC): R{rc1} P{rc2} T{rc3} Y{rc4} | vx{vx:.2f} vy{vy:.2f} vz{vz:.2f} yr{yr:.2f}")
                return

            # Ensure FC is in GUIDED mode so it follows our setpoints
            # (ArduPilot ignores position/velocity targets in STABILIZE/ALT_HOLD).
            # ONLY when there IS a movement setpoint — this used to run UNCONDITIONALLY, so any
            # non-numeric payload (a named command, gimbal-only, LED) force-flipped the FC to GUIDED
            # as a side effect (e.g. mid-LOITER hold, or during manual flight). Now it can't.
            if has_velocity or has_gps or has_local_pos:
                current_mode = self.telemetry_cache.get('mode_id', '')
                if str(current_mode) not in ('GUIDED', '4'):
                    self.fc.mav.set_mode_send(
                        self.fc.target_system,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                        4  # GUIDED
                    )

            # Yaw handling (works with any mode)
            yaw_deg = float(p.get('yaw', 0))
            yaw_rate = float(p.get('yaw_rate', 0))
            yaw_rad = math.radians(yaw_deg) if yaw_deg else 0

            if has_velocity:
                vx = float(p.get('vx', 0))
                vy = float(p.get('vy', 0))
                vz = float(p.get('vz', 0))

                # Safety clamp: 10 m/s max per axis
                clamp = lambda v, mx: max(-mx, min(mx, v))
                vx, vy, vz = clamp(vx, 10), clamp(vy, 10), clamp(vz, 10)

                # MA-29 FIX: Apply smart obstacle avoidance to AI velocities
                # Uses potential field method — smoothly steers around obstacles
                # instead of crude binary stop at 500mm
                if self.obstacle_avoidance:
                    tof_data = {
                        't1': self.telemetry_cache.get('t1', -1),
                        't2': self.telemetry_cache.get('t2', -1),
                        't3': self.telemetry_cache.get('t3', -1),
                        't4': self.telemetry_cache.get('t4', -1),
                    }
                    drone_yaw = self.telemetry_cache.get('yaw', 0)
                    vx, vy, vz = self.obstacle_avoidance.adjust_velocity_command(
                        vx, vy, vz, tof_data, drone_yaw_rad=drone_yaw
                    )

                type_mask = (
                    0b0000_0001_11_000_111  # ignore position, ignore accel
                )
                # If yaw_rate is set, use yaw rate; otherwise use yaw angle
                if yaw_rate:
                    type_mask |= (1 << 10)   # ignore yaw, use yaw_rate
                else:
                    type_mask |= (1 << 11)   # ignore yaw_rate, use yaw

                self.fc.mav.set_position_target_local_ned_send(
                    0,  # time_boot_ms
                    self.fc.target_system,
                    self.fc.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    type_mask,
                    0, 0, 0,           # x, y, z (ignored)
                    vx, vy, vz,        # velocity
                    0, 0, 0,           # accel (ignored)
                    yaw_rad,
                    math.radians(yaw_rate) if yaw_rate else 0
                )

            elif has_gps:
                lat = float(p['lat'])
                lng = float(p['lng'])
                alt = float(p.get('alt', 5))

                type_mask = (
                    0b0000_0001_11_111_000  # use position, ignore velocity & accel
                )
                if yaw_rate:
                    type_mask |= (1 << 10)
                else:
                    type_mask |= (1 << 11)

                self.fc.mav.set_position_target_global_int_send(
                    0,
                    self.fc.target_system,
                    self.fc.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    type_mask,
                    int(lat * 1e7),
                    int(lng * 1e7),
                    alt,
                    0, 0, 0,
                    0, 0, 0,
                    yaw_rad,
                    math.radians(yaw_rate) if yaw_rate else 0
                )

            elif has_local_pos:
                x = float(p.get('x', 0))
                y = float(p.get('y', 0))
                z = float(p.get('z', -5))  # NED: negative = up

                type_mask = (
                    0b0000_0001_11_111_000  # use position, ignore velocity & accel
                )
                if yaw_rate:
                    type_mask |= (1 << 10)
                else:
                    type_mask |= (1 << 11)

                self.fc.mav.set_position_target_local_ned_send(
                    0,
                    self.fc.target_system,
                    self.fc.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    type_mask,
                    x, y, z,
                    0, 0, 0,
                    0, 0, 0,
                    yaw_rad,
                    math.radians(yaw_rate) if yaw_rate else 0
                )

            # --- GIMBAL (if present alongside anything) ---
            if 'gimbal_pitch' in p or 'gimbal_yaw' in p or 'pan' in p:
                gp = float(p.get('gimbal_pitch', p.get('pitch', 0)))
                gy = float(p.get('gimbal_yaw', p.get('pan', 0)))
                await self.esp32_cmd_queue.put({"type": "gimbal", "pitch": gp, "yaw": gy})

            # --- MODE CHANGE (if AI decides to switch flight mode) ---
            if 'mode' in p:
                mode_name = str(p['mode']).upper()
                mode_map = {
                    'STABILIZE': 0, 'ACRO': 1, 'ALT_HOLD': 2,
                    'AUTO': 3, 'GUIDED': 4, 'LOITER': 5,
                    'RTL': 6, 'LAND': 9, 'POSHOLD': 16,
                }
                mode_id = mode_map.get(mode_name, int(p['mode']) if str(p['mode']).isdigit() else None)
                if mode_id is not None:
                    self.fc.mav.set_mode_send(
                        self.fc.target_system,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                        mode_id
                    )

            # --- ARM/DISARM (if AI decides) ---
            if 'arm' in p:
                arm_val = 1 if p['arm'] else 0
                self.fc.mav.command_long_send(
                    self.fc.target_system, self.fc.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, arm_val, 0, 0, 0, 0, 0, 0
                )

            # --- LED feedback via ESP32 ---
            if 'led' in p:
                await self.esp32_cmd_queue.put({"type": "led", "color": p['led']})

        except Exception as e:
            print(f"AI PLAN EXEC ERROR: {e}")

    async def start_local_server(self):
        print("🚀 STARTING LOCAL AI BRIDGE (0.0.0.0:8000)...")
        # Standard Websocket Server for Control/Telemetry
        for attempt in range(5):
            try:
                async with websockets.serve(self.handle_local_client, "0.0.0.0", 8000, reuse_port=True):
                    await asyncio.Future() # Run forever
            except OSError as e:
                print(f"⚠️ Port 8000 busy (attempt {attempt+1}/5): {e}")
                await asyncio.sleep(3)

    MEDIA_EXT = {'.mp4': 'video/mp4', '.jpg': 'image/jpeg', '.jpeg': 'image/jpeg', '.png': 'image/png'}

    def _media_dir(self):
        """Where video_loop writes flight_record_*.mp4 and photo_*.jpg (the process cwd)."""
        return os.path.abspath(os.environ.get('MEDIA_DIR', os.getcwd()))

    async def handle_media_list(self, request):
        """List media recorded on the aircraft, newest first.

        Returns the shape the application's gallery already expects:
            [{id, type: 'photo'|'video', url, name, size, timestamp}]
        """
        from aiohttp import web
        host = request.headers.get('Host') or f"{TAILSCALE_RADXA_IP}:8080"
        base = f"http://{host}/media/"
        items = []
        d = self._media_dir()
        try:
            for name in os.listdir(d):
                ext = os.path.splitext(name)[1].lower()
                if ext not in self.MEDIA_EXT:
                    continue
                if not (name.startswith('flight_record_') or name.startswith('photo_')):
                    continue
                fp = os.path.join(d, name)
                try:
                    st = os.stat(fp)
                except OSError:
                    continue
                items.append({
                    "id": name,
                    "name": name,
                    "type": "video" if ext == '.mp4' else "photo",
                    "url": base + name,
                    "size": st.st_size,
                    "timestamp": int(st.st_mtime),
                })
        except Exception as e:
            print(f"⚠️ media list failed: {e}")
        items.sort(key=lambda i: i["timestamp"], reverse=True)
        return web.json_response(items)

    async def handle_media_file(self, request):
        """Serve one recorded file. The name is validated against the listing rules and
        resolved inside the media directory, so it cannot escape via .. or an absolute path."""
        from aiohttp import web
        name = request.match_info.get('name', '')
        if os.path.basename(name) != name:
            return web.Response(status=400, text="bad name")
        ext = os.path.splitext(name)[1].lower()
        if ext not in self.MEDIA_EXT:
            return web.Response(status=404, text="not found")
        d = self._media_dir()
        fp = os.path.abspath(os.path.join(d, name))
        if os.path.dirname(fp) != d or not os.path.isfile(fp):
            return web.Response(status=404, text="not found")
        return web.FileResponse(fp, headers={"Content-Type": self.MEDIA_EXT[ext]})

    async def start_local_video_server(self):
        from aiohttp import web
        print("🎥 STARTING LOCAL VIDEO SERVER (0.0.0.0:8080)...")
        app = web.Application()
        app.router.add_get('/snapshot', self.handle_snapshot)
        app.router.add_get('/stream', self.handle_stream)
        # Media recorded ON THE AIRCRAFT was previously unreachable: the app asked the cloud
        # relay for /media while the files sat on this device, retrievable only over SSH.
        app.router.add_get('/media', self.handle_media_list)
        app.router.add_get('/media/{name}', self.handle_media_file)
        runner = web.AppRunner(app)
        await runner.setup()
        for attempt in range(5):
            try:
                site = web.TCPSite(runner, '0.0.0.0', 8080, reuse_port=True)
                await site.start()
                break
            except OSError as e:
                print(f"⚠️ Port 8080 busy (attempt {attempt+1}/5): {e}")
                await asyncio.sleep(3)
        # Keep alive
        while True:
            await asyncio.sleep(3600)

    async def handle_snapshot(self, request):
        if self.latest_jpeg:
            return web.Response(body=self.latest_jpeg, content_type='image/jpeg')
        return web.Response(status=503, text="No Frame Yet")

    async def handle_stream(self, request):
        resp = web.StreamResponse(
            status=200,
            reason='OK',
            headers={
                'Content-Type': 'multipart/x-mixed-replace;boundary=frame',
                'Cache-Control': 'no-store, no-cache, must-revalidate, pre-check=0, post-check=0, max-age=0',
                'Pragma': 'no-cache',
                'Expires': '0',
            }
        )
        await resp.prepare(request)
        try:
            while True:
                if self.latest_jpeg:
                    frame = self.latest_jpeg
                    try:
                        await resp.write(b'--frame\r\n')
                        await resp.write(b'Content-Type: image/jpeg\r\n\r\n')
                        await resp.write(frame)
                        await resp.write(b'\r\n')
                        await asyncio.sleep(0.04) # Max 25fps
                    except:
                        break
                else:
                    await asyncio.sleep(0.1)
        except:
            pass
        return resp

    async def handle_local_client(self, websocket, path=None):  # path optional: websockets v11+ omits it
        print("🔗 FAST-LINK: Client Connected (Local)")
        self.local_clients.add(websocket)
        try:
            async for message in websocket:
                # Direct packet injection or handshake ignore
                try:
                    data = json.loads(message)
                    # Handshake support (Director sends {id:..., token:...})
                    if 'token' in data:
                        print(f"🤝 FAST-LINK Handshake: {data.get('id')}")
                        continue
                    # RESTORED (wiped 6-30..7-02): INBOUND logging + the peer relay. Without the
                    # relay, the app's AI-box jobs (type 'ai_job') never reached the laptop director
                    # AND the laptop's ai_response/ai_status never reached the app — the whole
                    # natural-language AI command loop was dead over the local link.
                    _t = data.get('type')
                    if _t not in ('joystick', 'telemetry', 'user_gps', 'gimbal'):
                        _pl = data.get('payload')
                        _act = _pl.get('action') if isinstance(_pl, dict) else _pl
                        print(f"INBOUND type={_t} action={_act} payload={str(_pl)[:120]}")
                    # Relay AI / natural-language jobs to peer clients (laptop_vision director)
                    if _t in ('ai_job', 'neural', 'neural_command', 'ai', 'nl_command', 'job',
                              'text_command', 'ai_response', 'ai_status'):
                        print(f"RELAY ai_job -> {len(self.local_clients)-1} peer(s)")
                        for c in list(self.local_clients):
                            if c is not websocket:
                                try:
                                    await c.send(message)
                                except Exception:
                                    self.local_clients.discard(c)
                    # Process command
                    await self.process_packet(data.get('type'), data.get('payload'))
                except Exception as e:
                    print(f"⚠️ FAST-LINK Data Error: {e}")
        except Exception as e:
             print(f"🔗 FAST-LINK loop ended: {type(e).__name__}: {e}")
        finally:
             print("🔗 FAST-LINK: Client Disconnected")
             self.local_clients.remove(websocket)

    def init_esp32(self):
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.esp32_addr = ("192.168.4.2", 8888)
        print(f"🔭 ESP32 Gimbal Link Active -> {self.esp32_addr}")
    async def connect_mavlink(self):
        self.init_esp32()
        # lidar_loop() is started in main asyncio.gather(), NOT here (was causing double-start)
        # V110: Use configured baud rate (57600 for ArduPilot default)
        bauds = [FC_BAUD]
        while self.running:
            for baud in bauds:
                try:
                    print(f"🔌 Connecting to FC on {FC_PORT} @ {baud}...")
                    self.fc = mavutil.mavlink_connection(FC_PORT, baud=baud)
                    hb = self.fc.wait_heartbeat(timeout=2)
                    if hb is None:
                        print(f"❌ Heartbeat Timeout @ {baud}.")
                        self.fc.close()
                        continue
                    print(f"✅ FC Connected @ {baud}! Heartbeat receiving.")
                    self.fc.mav.request_data_stream_send(self.fc.target_system, self.fc.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2, 1)
                    self.fc.mav.request_data_stream_send(self.fc.target_system, self.fc.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 4, 1) # Attitude
                    self.fc.mav.request_data_stream_send(self.fc.target_system, self.fc.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 4, 1) # HUD
                    self.fc.mav.request_data_stream_send(self.fc.target_system, self.fc.target_component, mavutil.mavlink.MAV_DATA_STREAM_POSITION, 4, 1) # REL_ALT
                    self.fc.mav.request_data_stream_send(self.fc.target_system, self.fc.target_component, mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 4, 1) # RANGEFINDER ground-truth echo (Jul-2 diagnostic, kept)

                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component, b'ARMING_CHECK', 0, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
                    # RESTORED (wiped 6-30..7-02, orig patch_thr_dz 6-24):
                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component, b'THR_DZ', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT16)  # no ALT_HOLD throttle deadzone -> small climb cmds climb

                    # --- Obstacle-avoidance prerequisites -------------------------------
                    # PRX1_TYPE=2 (MAVLink proximity) is REQUIRED for ArduPilot to consume the
                    # 360-degree OBSTACLE_DISTANCE ring this bridge publishes. With the previous
                    # value (4 = RangeFinder) the firmware ignored the ring entirely and
                    # BendyRuler/Dijkstra avoidance never ran. Enforced here so it survives a
                    # reflash or a parameter reset instead of depending on a manual GCS edit.
                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component,
                                               b'PRX1_TYPE', 2, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
                    # AUTO_OPTIONS=3: bit0 allows arming in AUTO, bit1 allows the takeoff item to
                    # run without a pilot throttle raise. Without it an app-flown mission arms and
                    # then sits on the ground (SITL-confirmed).
                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component,
                                               b'AUTO_OPTIONS', 3, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
                    print("⚙️ FC params enforced: PRX1_TYPE=2, AUTO_OPTIONS=3")

                    # --- GEOFENCE: the only bound on a CONFIDENTLY WRONG command ------------
                    # Every other safety layer is obstacle-relative (clearance table, caution
                    # radius, reactive avoidance). In open air they all report "clear", so a
                    # wrong heading is flown at full commanded speed with nothing to stop it.
                    # ArduPilot enforces this fence itself, so it still holds if the companion
                    # or the laptop AI misbehaves entirely. Needs a position estimate to act.
                    _fence_r = float(os.environ.get('FENCE_RADIUS_M', '50'))
                    _fence_a = float(os.environ.get('FENCE_ALT_MAX_M', '30'))
                    _fence_on = os.environ.get('FENCE_ENABLE', '1') not in ('0', '', 'false', 'False')
                    if _fence_on:
                        for _pn, _pv, _pt in (
                            (b'FENCE_TYPE',   3,        mavutil.mavlink.MAV_PARAM_TYPE_INT8),   # alt + circle
                            (b'FENCE_RADIUS', _fence_r, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
                            (b'FENCE_ALT_MAX',_fence_a, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
                            (b'FENCE_ACTION', 1,        mavutil.mavlink.MAV_PARAM_TYPE_INT8),   # 1 = RTL
                            (b'FENCE_ENABLE', 1,        mavutil.mavlink.MAV_PARAM_TYPE_INT8),
                        ):
                            self.fc.mav.param_set_send(self.fc.target_system,
                                                       self.fc.target_component, _pn, _pv, _pt)
                        print(f"🚧 GEOFENCE enforced: radius={_fence_r}m alt_max={_fence_a}m "
                              f"action=RTL (set FENCE_ENABLE=0 to disable)")
                    else:
                        print("⚠️ GEOFENCE DISABLED by FENCE_ENABLE=0 - nothing bounds a wrong heading")

                    # Read back the rest of the avoidance chain and report it. These are NOT
                    # written automatically -- they are reported so a wrong value is visible
                    # rather than silently disabling avoidance in flight.
                    for _p in (b'PRX1_TYPE', b'AUTO_OPTIONS', b'AVOID_ENABLE', b'AVOID_BEHAVE',
                               b'AVOID_MARGIN', b'OA_TYPE', b'RNGFND1_TYPE', b'BATT_VOLT_MULT'):
                        self.fc.mav.param_request_read_send(self.fc.target_system,
                                                            self.fc.target_component, _p, -1)
                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component, b'DISARM_DELAY', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT8)  # never auto-disarm mid-sequence
                    # APP-DRIVEN AUTO MISSIONS (SITL-found 2026-07-12): with default AUTO_OPTIONS,
                    # a mission in AUTO waits for a PILOT THROTTLE RAISE before the takeoff item —
                    # an app-flown drone has no throttle stick, so the route missions would sit
                    # armed and idle forever. Bits: 1=allow arming in AUTO, 2=allow AUTO takeoff
                    # without raising throttle.
                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component, b'AUTO_OPTIONS', 3, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component, b'EKF2_GPS_CHECK', 0, mavutil.mavlink.MAV_PARAM_TYPE_UINT32) # V26: Kill EKF GPS Check
                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component, b'FS_EKF_THRESH', 0, mavutil.mavlink.MAV_PARAM_TYPE_UINT32) # V26: Kill EKF Failsafe

                    # V27: AUTO-CONFIGURE FOR INDOOR (Disable GPS & Failsafes)
                    # GPS_TYPE=0 removed — contradicts V106 GPS enable below (GPS_TYPE=1)
                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component, b'AHRS_GPS_USE', 0, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component, b'FS_THR_ENABLE', 0, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component, b'FS_GCS_ENABLE', 0, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
                    # self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component, b'FS_BATT_ENABLE', 0, mavutil.mavlink.MAV_PARAM_TYPE_UINT32) # V46: Re-enabled below
                    # V106: ENABLE GPS (User Req: "GPS needs to be enabled")
                    # EKF3 uses GPS + Compass + IMU.
                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component, b'AHRS_EKF_TYPE', 3, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component, b'GPS_TYPE', 1, mavutil.mavlink.MAV_PARAM_TYPE_UINT32) # 1=Auto

                    # V39: UNLOCKED PRECISION MODE (User Request: "Exact Joystick Control")
                    # disabling Deadzones so even micro-movements are registered
                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component, b'RC1_DZ', 0, mavutil.mavlink.MAV_PARAM_TYPE_UINT16)
                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component, b'RC2_DZ', 0, mavutil.mavlink.MAV_PARAM_TYPE_UINT16)
                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component, b'RC3_DZ', 0, mavutil.mavlink.MAV_PARAM_TYPE_UINT16) # Throttle Deadzone = 0
                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component, b'RC4_DZ', 0, mavutil.mavlink.MAV_PARAM_TYPE_UINT16)

                    # Restoring Speed Limits to Standard (User wants full authority)
                    # PILOT_SPEED_UP=250 removed — overridden by V72 PILOT_SPEED_UP=500 below
                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component, b'PILOT_SPEED_DN', 150, mavutil.mavlink.MAV_PARAM_TYPE_UINT16)
                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component, b'PILOT_ACCEL_Z', 250, mavutil.mavlink.MAV_PARAM_TYPE_UINT16)

                    # V42: ADAPTIVE HOVER LEARNING (User Request: "Adapt to 1.5kg Weight")
                    # Enable Hover Learning (2=Learn and Save). 0.55 for 1.5kg F450.
                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component, b'MOT_THST_HOVER', 0.55, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                    # V53: MOTOR IDLE FIX — MOT_SPIN_ARM=0.12 and MOT_SPIN_MIN=0.15 removed
                    # They were overridden by V72 values (0.25/0.25) below
                    # V104: BATTERY CALIBRATION (Standard MiniPix Defaults)
                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component, b'BATT_MONITOR', 4, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component, b'BATT_VOLT_PIN', 2, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component, b'BATT_CURR_PIN', 3, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component, b'BATT_VOLT_MULT', 4.4012, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component, b'BATT_AMP_PERVOLT', 18.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                    # Battery: 5400mAh 3S 60C LiPo
                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component, b'BATT_CAPACITY', 5400, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component, b'ANGLE_MAX', 6000, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component, b'PILOT_SPEED_UP', 500, mavutil.mavlink.MAV_PARAM_TYPE_INT16)
                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component, b'MOT_SPOOL_TIME', 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32) # V72: ZERO DELAY
                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component, b'MOT_SPIN_ARM', 0.25, mavutil.mavlink.MAV_PARAM_TYPE_REAL32) # V72: Hot Idle
                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component, b'MOT_SPIN_MIN', 0.25, mavutil.mavlink.MAV_PARAM_TYPE_REAL32) # V72: Match Arm
                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component, b'ARMING_CHECK', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32) # V72: No Checks
                    # V71: Li-Ion Voltage Range (Monitor 4 + Python Override)
                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component, b'BATT_LOW_VOLT', 9.6, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component, b'BATT_CRT_VOLT', 9.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                    print("🔓 INDOOR MODE: ZERO DELAY (V72) | SMOOTHED BATTERY")
                    # Seed the movement-envelope (fc_caps) from what we set, then REQUEST the live values so
                    # PARAM_VALUE refreshes them — so the laptop AI bounds movement by the REAL FC config.
                    self.telemetry_cache.setdefault('fc_caps', {}).update({
                        'ANGLE_MAX': 6000.0, 'PILOT_SPEED_UP': 500.0, 'PILOT_SPEED_DN': 150.0,
                        'WPNAV_SPEED': 1000.0, 'PILOT_ACCEL_Z': 250.0,
                    })
                    for _p in (b'ANGLE_MAX', b'WPNAV_SPEED', b'PILOT_SPEED_UP', b'PILOT_SPEED_DN', b'PILOT_ACCEL_Z'):
                        try:
                            self.fc.mav.param_request_read_send(self.fc.target_system, self.fc.target_component, _p, -1)
                        except Exception:
                            pass
                    return
                except Exception as e:
                    print(f"⚠️ FC Check Failed: {e}")
                    await asyncio.sleep(1)
            print("🔴 FC NOT DETECTED. Retrying...")
            await asyncio.sleep(2)
    async def connect_cloud(self):
        # The control path is LOCAL by design. The bridge used to dial the public relay on
        # every boot regardless, which both exposed the aircraft to an internet service and
        # burned bandwidth on the flight link. Opt in explicitly if the relay is wanted.
        if os.environ.get("ENABLE_CLOUD_RELAY", "0") in ("0", "", "false", "False"):
            print("[LINK] Cloud relay DISABLED (set ENABLE_CLOUD_RELAY=1 to enable). Local link only.")
            return
        while self.running:
            try:
                print(f"☁️ Connecting to Cloud: {SERVER_URL}...")
                # User Request: "NEVER disconnect on its own" -> INFINITE TIMEOUT
                async with websockets.connect(SERVER_URL, ping_interval=10, ping_timeout=None) as ws:
                    self.ws = ws
                    print("✅ Cloud Connected!")

                    # AUTH FRAME
                    auth_frame = {
                        "type": "connect_drone",
                        "droneId": "RADXA_X",
                        "token": "bearer_token"
                    }
                    await ws.send(json.dumps(auth_frame))


                    # Command loop runs inside cloud connection (needs self.ws)
                    # Telemetry loop runs SEPARATELY in main gather (doesn't need cloud)
                    await self.command_loop()
                    self.fc.mav.request_data_stream_send(self.fc.target_system, self.fc.target_component, mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 2, 1) # V27: Debug RC
            except Exception as e:
                print(f"⚠️ Cloud Disconnected: {e}. Retrying in 5s...")
                self.ws = None
                await asyncio.sleep(5)
    async def telemetry_loop(self):
        last_send = 0
        telem_count = 0
        while self.running:  # Don't depend on self.ws — telemetry must flow even if cloud is down
            if self.fc:
                # V22: Drain Buffer (Process up to 50 msgs per loop to catch ACKs)
                try:
                  for _ in range(50):
                    msg = self.fc.recv_match(blocking=False)
                    if not msg: break
                    # Liveness stamp. Nothing previously noticed the flight controller going
                    # quiet (unplugged lead, brownout, UART fault): self.fc stayed set and the
                    # telemetry cache kept serving its LAST values forever, so the battery
                    # failsafe could never fire and armed/altitude/gps_fix stayed frozen.
                    self._fc_msg_t = time.time()
                    if getattr(self, '_fc_link_lost', False):
                        self._fc_link_lost = False
                        print("✓ FC link restored")

                    type = msg.get_type()
                    # Battery Failsafe
                    if type == 'SYS_STATUS':
                        # V70: FORCE PYTHON CALCULATION (If FC fails or is stuck)
                        batt_pct = msg.battery_remaining
                        batt_voltage = msg.voltage_battery / 1000.0
                        if batt_pct < 0 or batt_pct > 95:
                             # V71: Li-Ion Curve (9.0V to 12.6V)
                             calc_pct = int((batt_voltage - 9.0) / 3.6 * 100.0)
                             batt_pct = max(0, min(100, calc_pct))

                        # Real LiPo 3S discharge curve (per-cell voltage → %)
                        cell_v = batt_voltage / 3.0
                        # Measured LiPo discharge curve lookup (voltage per cell → capacity %)
                        LIPO_CURVE = [
                            (4.20, 100), (4.15, 95), (4.11, 90), (4.08, 85),
                            (4.02, 80), (3.98, 75), (3.95, 70), (3.91, 65),
                            (3.87, 60), (3.85, 55), (3.84, 50), (3.82, 45),
                            (3.80, 40), (3.79, 35), (3.77, 30), (3.75, 25),
                            (3.73, 20), (3.71, 15), (3.69, 10), (3.61, 5),
                            (3.27, 0),
                        ]
                        if cell_v >= 4.20:
                            batt_pct = 100
                        elif cell_v <= 3.27:
                            batt_pct = 0
                        else:
                            for i in range(len(LIPO_CURVE) - 1):
                                v_hi, p_hi = LIPO_CURVE[i]
                                v_lo, p_lo = LIPO_CURVE[i + 1]
                                if v_lo <= cell_v <= v_hi:
                                    batt_pct = int(p_lo + (cell_v - v_lo) / (v_hi - v_lo) * (p_hi - p_lo))
                                    break
                        self.telemetry_cache['battery'] = batt_pct
                        self.telemetry_cache['cell_voltage'] = round(cell_v, 2)
                        self.telemetry_cache['voltage'] = batt_voltage
                        # V133_REAL_ARMED (restored — was wiped): 'armed' now comes from
                        # HEARTBEAT.base_mode (the REAL arm state) in the HEARTBEAT handler below.
                        # The old gyro-health bit was ALWAYS 1, so the laptop "dreamed" liftoff
                        # while the motors never moved.

                        # V26: Capture Mode
                        self.telemetry_cache['mode_id'] = self.fc.flightmode

                        if msg.battery_remaining < self.batt_threshold and not self.low_batt_triggered:
                            print(f"LOW BATT < {self.batt_threshold}%! Smart RTH -> {self.batt_rth_destination}")
                            self.low_batt_triggered = True
                            self.follow_me_active = False  # Stop follow on low batt
                            # RESTORED (wiped): GPS gate — low-batt return without a 3D fix is a
                            # blind RTL -> LAND in place instead (safe, indoors-correct).
                            _fix = int(self.telemetry_cache.get('gps_fix', 0) or 0)
                            if _fix < 3:
                                print(f"LOW BATT: no GPS (fix={_fix}) — LANDING IN PLACE (safe, no blind RTL)")
                                self.fc.mav.set_mode_send(self.fc.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 9)
                            elif self.batt_rth_destination == 'user' and self.user_gps:
                                lat, lng = self.user_gps
                                print(f"RTH to User: {lat}, {lng}")
                                self.fc.mav.set_mode_send(self.fc.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4)
                                self.fc.mav.mission_item_int_send(self.fc.target_system, self.fc.target_component, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 0, 0, 0, 0, int(lat * 1e7), int(lng * 1e7), 15)
                            else:
                                print("RTH to Launch")
                                self.fc.set_mode('RTL')

                    elif type == 'STATUSTEXT': # V23: The Voice of the FC
                         print(f"📢 FC SAYS: {msg.text}")
                         # V131: Push FC text messages (AutoTune Success/Failed, arming errors, etc.)
                         # live to the laptop over the SAME Tailscale local_clients broadcast used for
                         # telemetry — so you see it on the laptop with no wire/telemetry radio needed.
                         if self.local_clients:
                             status_msg = json.dumps({
                                 "type": "fc_status_text",
                                 "payload": {"text": msg.text, "severity": int(msg.severity), "ts": time.time()}
                             })
                             for c in list(self.local_clients):
                                 try:
                                     await asyncio.wait_for(c.send(status_msg), timeout=2.0)
                                 except Exception:
                                     pass

                    elif type == 'HEARTBEAT':
                        # V133_REAL_ARMED (restored): TRUE armed state (motors hot) straight from the FC.
                        self.telemetry_cache['armed'] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

                    elif type == 'PARAM_VALUE': # V23: TX Verification + live FC capability envelope
                         _pid = msg.param_id
                         if not isinstance(_pid, str):
                             try: _pid = _pid.decode('utf-8', 'ignore')
                             except Exception: _pid = str(_pid)
                         _pid = _pid.strip('\x00 ')
                         # Forward the movement-envelope params to the laptop as fc_caps so the AI's movement
                         # is bounded by what THIS airframe is ACTUALLY configured for (live, not hardcoded).
                         if _pid in ('ANGLE_MAX', 'WPNAV_SPEED', 'PILOT_SPEED_UP', 'PILOT_SPEED_DN',
                                     'PILOT_ACCEL_Z', 'WPNAV_ACCEL', 'ATC_SLEW_YAW'):
                             self.telemetry_cache.setdefault('fc_caps', {})[_pid] = float(msg.param_value)
                             print(f"⚙️ fc_caps {_pid} = {msg.param_value}")
                         else:
                             print(f"✅ TX VERIFIED: Read Param {_pid} = {msg.param_value}")  # V23 (kept)
                    elif type in ('MISSION_REQUEST', 'MISSION_REQUEST_INT'):
                         # PROPER mission-upload handshake (the old cloud path blind-blasted items;
                         # ArduPilot pulls them by seq — answer each request from the pending list).
                         _pm = getattr(self, '_pending_mission', None)
                         if _pm and 0 <= msg.seq < len(_pm):
                             it = _pm[msg.seq]
                             self.fc.mav.mission_item_int_send(
                                 self.fc.target_system, self.fc.target_component, msg.seq,
                                 mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                 it['cmd'], 0, 1, 0, 0, 0, 0,
                                 int(it['lat'] * 1e7), int(it['lng'] * 1e7), float(it['alt']))
                    elif type == 'MISSION_ACK':
                         _pm = getattr(self, '_pending_mission', None)
                         if _pm:
                             _n = len(_pm) - 2
                             print(f"🗺️ MISSION_ACK type={msg.type} — {_n} waypoints on the FC")
                             self._pending_mission = None
                             _msg = {"type": "alert", "payload": {
                                 "msg": f"MISSION UPLOADED ({_n} waypoints)" if msg.type == 0 else
                                        f"MISSION REJECTED by FC (MAV_MISSION result {msg.type})",
                                 "level": "info" if msg.type == 0 else "error"}}
                             for c in list(self.local_clients):
                                 try: await c.send(json.dumps(_msg))
                                 except Exception: pass
                             if msg.type == 0 and getattr(self, '_mission_autostart', False):
                                 self._mission_autostart = False
                                 if self.is_armed:
                                     print("🗺️ Mission accepted + ARMED -> switching AUTO (mission flies)")
                                     self.fc.mav.set_mode_send(self.fc.target_system,
                                         mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 3)
                                 else:
                                     print("🗺️ Mission accepted (DISARMED) — ARM then switch AUTO to fly it")
                    elif type == 'WIND':
                         # ArduPilot EKF 2D wind estimate — the AI shows it (info); the FC's mode does the
                         # actual wind rejection (GUIDED/Loiter/PosHold with GPS).
                         self.telemetry_cache['wind_dir'] = getattr(msg, 'direction', None)
                         self.telemetry_cache['wind_speed'] = getattr(msg, 'speed', None)
                    elif type == 'ATTITUDE':
                         self.telemetry_cache['roll'] = msg.roll
                         self.telemetry_cache['pitch'] = msg.pitch
                         self.telemetry_cache['yaw'] = msg.yaw
                         # V110: Derive heading from yaw (0-360 degrees)
                         import math
                         heading_deg = math.degrees(msg.yaw) % 360
                         self.telemetry_cache['heading'] = heading_deg

                    elif type == 'GLOBAL_POSITION_INT':
                         # V110: Extract GPS position, altitude, speed, heading
                         self.telemetry_cache['lat'] = msg.lat / 1e7
                         self.telemetry_cache['lng'] = msg.lon / 1e7
                         self.telemetry_cache['altitude_gps'] = msg.relative_alt / 1000.0  # mm to m
                         self.telemetry_cache['raw_alt'] = msg.alt / 1000.0  # MSL altitude
                         # Ground speed from vx/vy (cm/s to m/s)
                         vx = msg.vx / 100.0
                         vy = msg.vy / 100.0
                         self.telemetry_cache['speed'] = (vx**2 + vy**2)**0.5
                         # Keep the COMPONENTS too, not just the magnitude. The SLAM pose estimator
                         # wants a dead-reckoning prior and was being handed a hardcoded (0,0), so
                         # its hill-climb had to re-find the position from scratch on every scan
                         # instead of starting from where the aircraft was actually heading.
                         self.telemetry_cache['vel_n'] = vx      # NED north component (m/s)
                         self.telemetry_cache['vel_e'] = vy      # NED east component (m/s)
                         # Heading from GPS (cdeg to deg)
                         self.telemetry_cache['heading'] = msg.hdg / 100.0 if msg.hdg != 65535 else self.telemetry_cache.get('heading', 0)

                    elif type == 'GPS_RAW_INT':
                         # V110: Satellite count + GPS fix quality
                         self.telemetry_cache['sats'] = msg.satellites_visible
                         self.telemetry_cache['gps_fix'] = msg.fix_type  # 0=no, 2=2D, 3=3D
                         # Jul-2 diagnostic (kept): periodic human-readable GPS status in the journal —
                         # essential for the new-GPS outdoor bench test (watch fix go NO_FIX->3D_FIX).
                         self._dbg_gps_count = getattr(self, '_dbg_gps_count', 0) + 1
                         if self._dbg_gps_count % 10 == 1:
                             _fixnames = {0: "NO_GPS", 1: "NO_FIX", 2: "2D_FIX", 3: "3D_FIX", 4: "DGPS", 5: "RTK_FLOAT", 6: "RTK_FIXED"}
                             print(f"🛰️ GPS STATUS: fix={_fixnames.get(msg.fix_type, msg.fix_type)} sats={msg.satellites_visible} hdop={msg.eph/100.0 if msg.eph!=65535 else 'N/A'}")

                    elif type == 'RANGEFINDER':
                         # Jul-2 diagnostic (kept): the FC ECHOES its internal rangefinder value — proves
                         # whether the FC is INGESTING our LiDAR DISTANCE_SENSOR (the rangefinder1 check
                         # without Mission Planner).
                         print(f"🎯 FC RANGEFINDER ECHO: distance={msg.distance}m voltage={msg.voltage}")

                    elif type == 'DISTANCE_SENSOR':
                         print(f"🎯 FC DISTANCE_SENSOR ECHO: current_distance={msg.current_distance}cm id={msg.id} orient={msg.orientation}")

                    elif type == 'VFR_HUD':
                         # V110: Airspeed, groundspeed, altitude, climb rate
                         self.telemetry_cache['speed'] = msg.groundspeed
                         self.telemetry_cache['altitude_baro'] = msg.alt
                         self.telemetry_cache['climb_rate'] = msg.climb
                         if msg.heading != 65535:
                             self.telemetry_cache['heading'] = msg.heading

                    elif type == 'RC_CHANNELS_RAW': # V27: Debug Switch Positions
                         # Log channels 5, 6, 7 (common for mode switches) every 2s
                         if int(time.time()) % 2 == 0 and int(time.time()) != getattr(self, 'last_rc_log', 0):
                              self.last_rc_log = int(time.time())
                              print(f"🎮 RC RAW: C5={msg.chan5_raw} C6={msg.chan6_raw} C7={msg.chan7_raw}")
                    # DATA FUSION Logic
                    sats = self.telemetry_cache.get('sats', 0)
                    if sats > 5:
                         self.telemetry_cache['altitude'] = self.telemetry_cache.get('altitude_gps', 0)
                         self.telemetry_cache['source'] = 'GPS'
                    else:
                         self.telemetry_cache['altitude'] = self.telemetry_cache.get('altitude_baro', 0)
                         self.telemetry_cache['source'] = 'BARO'

                    # V110: Calculate distance from home (for app display)
                    lat = self.telemetry_cache.get('lat', 0)
                    lng = self.telemetry_cache.get('lng', 0)
                    if lat != 0 and lng != 0:
                        if not hasattr(self, 'home_lat') or self.home_lat is None:
                            self.home_lat = lat
                            self.home_lng = lng
                        import math
                        dlat = math.radians(lat - self.home_lat)
                        dlng = math.radians(lng - self.home_lng)
                        a = math.sin(dlat/2)**2 + math.cos(math.radians(self.home_lat)) * math.cos(math.radians(lat)) * math.sin(dlng/2)**2
                        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
                        self.telemetry_cache['distance'] = round(6371000 * c, 1)  # meters
                except Exception:
                    await asyncio.sleep(2)
                    continue
            now = time.time()
            # P3.0: LiDAR data is injected by lidar_loop() directly into telemetry_cache
            # P3.1: Inject safety status into telemetry
            self.telemetry_cache['safety'] = self.safety.get_safety_status()

            # FOLLOW ME: Continuously send user GPS as GUIDED waypoint (every 2s)
            if self.follow_me_active and self.user_gps and self.fc:
                if not hasattr(self, '_last_follow_send') or now - self._last_follow_send > 2.0:
                    self._last_follow_send = now
                    lat, lng = self.user_gps
                    alt = self.telemetry_cache.get('altitude', 5)
                    alt = max(3, alt)  # Don't descend below 3m while following
                    # Ensure GUIDED mode
                    self.fc.mav.set_mode_send(
                        self.fc.target_system,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4
                    )
                    self.fc.mav.mission_item_int_send(
                        self.fc.target_system, self.fc.target_component,
                        0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                        2, 0, 0, 0, 0, 0,
                        int(lat * 1e7), int(lng * 1e7), alt
                    )
            if self.telemetry_cache:
                if now - last_send > 0.1: # 10Hz
                    try:
                        msg = json.dumps({"type": "telemetry", "payload": self.telemetry_cache})
                        telem_count += 1
                        sent_to = []
                        if self.ws:
                            try:
                                await asyncio.wait_for(self.ws.send(msg), timeout=2.0)
                                sent_to.append("cloud")
                            except Exception as e:
                                if telem_count % 50 == 0:  # Don't spam
                                    print(f"⚠️ Cloud telem send failed: {e}")
                        # P2.6: Broadcast to Local AI Clients (Tailscale direct)
                        if self.local_clients:
                            dead = set()
                            for c in self.local_clients:
                                try:
                                    await asyncio.wait_for(c.send(msg), timeout=5.0)
                                    sent_to.append("local")
                                except asyncio.TimeoutError:
                                    sent_to.append("local-slow")  # slow phone over Tailscale — KEEP it, don't drop
                                except Exception:
                                    dead.add(c)  # connection genuinely closed
                            self.local_clients -= dead
                    except Exception as e:
                        print(f"⚠️ Telemetry broadcast error: {e}")
                    last_send = now
                    # Diagnostic: show telemetry is flowing (every 5 seconds)
                    if telem_count % 50 == 1:
                        batt = self.telemetry_cache.get('battery', '?')
                        alt = self.telemetry_cache.get('altitude', '?')
                        print(f"📡 TELEM #{telem_count} → {sent_to} | Batt={batt}% Alt={alt}m")
                # DASHBOARD LOGGING (Every 2s)
                if int(now) % 2 == 0 and int(now) != getattr(self, 'last_log_sec', 0):
                    self.last_log_sec = int(now)
                    alt = self.telemetry_cache.get('altitude', 0)
                    src = self.telemetry_cache.get('source', 'UNK')
                    batt = self.telemetry_cache.get('battery', 0)
                    volt = self.telemetry_cache.get('voltage', 0)
                    sats = self.telemetry_cache.get('sats', 0)
                    raw = self.telemetry_cache.get('raw_alt', 0)
                    is_armed = self.fc.motors_armed() if self.fc else False
                    # Notify ESP32 when armed state changes (LED sync)
                    if is_armed != self.is_armed:
                        try:
                            self.esp32_cmd_queue.put_nowait({"armed": is_armed})
                        except: pass
                    self.is_armed = is_armed # V107: Update state
                    mode = self.telemetry_cache.get('mode_id', 'UNK')
                    est_cells = 3  # Hardcoded: 3S 5400mAh LiPo (auto-detect unreliable at low voltage)
                    speed = self.telemetry_cache.get('speed', 0)
                     # PRINT NEWLINE logs for debugging
                    print(f"📊 DATA: Alt={alt:.1f}m (Raw={raw:.1f}) | Batt={batt}% ({volt:.1f}V~{est_cells}S) | Mode={mode} | Spd={speed:.1f} | Armed={is_armed}")



                    # Force STABILIZE when disarmed (every 10s, not every 2s)
                    if not is_armed and mode != 'STABILIZE' and int(now) % 10 == 0 and int(now) != getattr(self, '_last_stab_force', 0):
                        self._last_stab_force = int(now)
                        self.fc.mav.set_mode_send(self.fc.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0)
                        self._auto_armed_done = False  # Reset auto-arm flag when disarmed

            await asyncio.sleep(0.01)
    async def _land_backstop_disarm(self):
        # RESTORED (wiped 6-30..7-02). After the LAND descent window, force-disarm so motors ALWAYS
        # stop (belt-and-suspenders, in case the landing detector doesn't fire). A redundant disarm
        # on the ground is harmless.
        await asyncio.sleep(15.0)
        if self.fc:
            for _ in range(5):
                self.fc.mav.command_long_send(self.fc.target_system, self.fc.target_component,
                                              mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                              0, 0, 21196, 0, 0, 0, 0, 0)
            print("LAND backstop: force-disarm after descent window")

    async def _goto_then_land(self, delay_s=10.0):
        # RESTORED (wiped 6-30..7-02). LAND "return to user": after the goto window, switch to LAND
        # so the drone descends at the user. ArduPilot's landing detector disarms on touchdown.
        await asyncio.sleep(delay_s)
        if self.fc:
            self.fc.mav.set_mode_send(self.fc.target_system,
                                      mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 9)
            print("LAND(user): goto window elapsed -> LAND mode for descent")

    async def _execute_settings_command(self, cmd, payload=None):
        """Apply an operator settings command. Single implementation shared by the local
        link and the cloud command loop.

        These previously existed ONLY inside the cloud command_loop, so over the local link
        (the path the app and laptop actually use) every settings command fell through to
        execute_ai_plan and became a silent no-op -- including the return-to-home destination
        and the battery threshold that drive the auto-return failsafe.

        Returns True if the command was a settings command and was handled.
        """
        payload = payload if isinstance(payload, dict) else {}

        # SET_CONFIG:key=value -- the app's settings panel format. The VALUE must keep its
        # original case ('user', '4k'); only the command token is upper-cased by the caller.
        if cmd.startswith('SET_CONFIG:'):
            config_str = cmd[len('SET_CONFIG:'):]
            if '=' not in config_str:
                print(f"⚠️ Malformed SET_CONFIG: {config_str}")
                return True
            key, val = config_str.split('=', 1)
            cfg = {key.strip().lower(): val.strip()}
            print(f"⚙️ SET_CONFIG: {cfg}")
            return await self._execute_settings_command('UPDATE_CONFIG', {'config': cfg})

        if cmd == 'UPDATE_CONFIG':
            cfg = payload.get('config', {}) or {}
            print(f"⚙️ SETTINGS UPDATED: {cfg}")
            if 'rth_behavior' in cfg:
                self.batt_rth_destination = 'user' if str(cfg['rth_behavior']).lower() == 'user' else 'launch'
                print(f"⚙️ RTH DEST -> {self.batt_rth_destination}")
            if 'land_behavior' in cfg:
                self.land_destination = 'user' if str(cfg['land_behavior']).lower() == 'user' else 'here'
                print(f"⚙️ LAND DEST -> {self.land_destination}")
            for k in ('batt_threshold', 'return_battery_pct', 'rth_battery', 'low_battery_pct'):
                if k in cfg:
                    try:
                        self.batt_threshold = max(5, min(50, int(float(cfg[k]))))
                        self.return_battery_pct = self.batt_threshold
                        self.low_batt_triggered = False
                        self._auto_return_done = False
                        print(f"⚙️ BATT THRESHOLD -> {self.batt_threshold}%")
                    except Exception:
                        pass
            return True

        if cmd in ('CALIBRATE_BATTERY', 'SET_BATT_CALIB'):
            # The pack reads ~27 V on a 3S (should be ~11-12 V), and that same voltage feeds the
            # auto-return threshold. Supply the voltage measured at the pack and the correct
            # multiplier is computed from what the FC currently reports:
            #     new_mult = old_mult * (measured / reported)
            try:
                measured = float(payload.get('measured_v', payload.get('voltage', 0)))
            except Exception:
                measured = 0.0
            reported = float(self.telemetry_cache.get('voltage') or 0)
            old_mult = float((self.telemetry_cache.get('fc_caps') or {}).get('BATT_VOLT_MULT') or 0)
            if measured <= 0:
                print("⚠️ CALIBRATE_BATTERY: supply measured_v (voltage at the pack)")
            elif reported <= 0:
                print("⚠️ CALIBRATE_BATTERY: FC is not reporting a voltage yet")
            elif old_mult <= 0:
                print("⚠️ CALIBRATE_BATTERY: BATT_VOLT_MULT unknown - requesting it, retry shortly")
                if self.fc:
                    self.fc.mav.param_request_read_send(self.fc.target_system,
                                                        self.fc.target_component, b'BATT_VOLT_MULT', -1)
            else:
                new_mult = old_mult * (measured / reported)
                print(f"🔋 BATT CALIB: reported={reported:.2f}V measured={measured:.2f}V "
                      f"mult {old_mult:.4f} -> {new_mult:.4f}")
                if self.fc:
                    self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component,
                                               b'BATT_VOLT_MULT', new_mult,
                                               mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                    self.fc.mav.param_request_read_send(self.fc.target_system,
                                                        self.fc.target_component, b'BATT_VOLT_MULT', -1)
            return True

        if cmd == 'SET_BATT_THRESHOLD':
            try:
                t = int(float(payload.get('threshold', 20)))
            except Exception:
                t = 20
            self.batt_threshold = max(5, min(50, t))
            self.return_battery_pct = self.batt_threshold
            self._auto_return_done = False
            print(f"⚙️ BATT THRESHOLD: {self.batt_threshold}%")
            return True

        if cmd in ('SET_BATT_RTH', 'SET_RTH_BEHAVIOR'):
            dest = str(payload.get('destination', payload.get('behavior', 'launch'))).lower()
            self.batt_rth_destination = 'user' if dest == 'user' else ('land' if dest == 'land' else 'launch')
            print(f"⚙️ RTH DEST: {self.batt_rth_destination}")
            return True

        if cmd == 'SET_RTH_ALT':
            try:
                alt_cm = int(float(payload.get('alt', 5000)))
            except Exception:
                alt_cm = 5000
            print(f"⚙️ RTH ALT: {alt_cm/100.0}m")
            if self.fc:
                self.fc.mav.param_set_send(
                    self.fc.target_system, self.fc.target_component,
                    b'RTL_ALT', alt_cm, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            return True

        if cmd == 'SET_SAFETY_CONFIG':
            oa = payload.get('obstacle_avoidance', True)
            vp = payload.get('vision_pos', True)
            self.telemetry_cache['obstacle_avoidance'] = oa
            self.telemetry_cache['vision_pos'] = vp
            print(f"⚙️ SAFETY CONFIG: obstacle_avoidance={oa}, vision_pos={vp}")
            return True

        if cmd == 'SWITCH_CAMERA':
            target = str(payload.get('camera', 'internal')).lower()
            self.active_cam = 'external' if target in ('external', 'gopro') else 'internal'
            self.telemetry_cache['active_cam'] = self.active_cam
            print(f"📷 CAMERA SWITCH: active={self.active_cam}")
            return True

        if cmd == 'SET_RECORDING_CAMERA':
            target = str(payload.get('camera', 'internal')).lower()
            self.recording_cam = 'external' if target in ('external', 'gopro') else 'internal'
            print(f"🎬 RECORDING CAMERA: {self.recording_cam}")
            return True

        if cmd == 'GOPRO_SETTINGS':
            self.gopro_settings = {**getattr(self, 'gopro_settings', {}), **payload}
            print(f"🎥 GOPRO SETTINGS: {payload}")
            # Actually execute it. gopro_proxy.handle_remote_ai_command already accepts this
            # exact {'action': ..., 'value': ...} shape -- it simply was never called, so every
            # GoPro button in the app was inert.
            if self.gopro_proxy:
                try:
                    await self.gopro_proxy.handle_remote_ai_command(payload)
                except Exception as e:
                    print(f"❌ GoPro command failed: {e}")
            else:
                print("⚠️ GoPro proxy not connected - command dropped")
            return True

        if cmd in ('CAPTURE', 'CAPTURE_PHOTO'):
            print("📸 CAPTURE PHOTO")
            if hasattr(self, 'esp32_cmd_queue') and self.esp32_cmd_queue:
                await self.esp32_cmd_queue.put({'type': 'capture'})
            return True

        if cmd in ('START_RECORDING', 'STOP_RECORDING', 'RECORD'):
            on = cmd != 'STOP_RECORDING'
            self.is_recording = on
            self.telemetry_cache['recording'] = on
            print(f"🔴 RECORDING: {'ON' if on else 'OFF'} (cam={getattr(self, 'recording_cam', 'internal')})")
            return True

        return False

    async def _execute_named_command(self, cmd, cmd_payload=None):
        """Execute a NAMED flight command over the LOCAL link with the SAME MAVLink actions the cloud
        command_loop uses (LAND smart-disarm, RTL/RTH incl. batt_rth_destination, RETURN_TO_USER,
        ARM split-arm, DISARM force, TAKEOFF, bare mode names). Returns True if the command was handled
        (caller then does NOT fall through to execute_ai_plan). Keep in sync with command_loop."""
        p = cmd_payload if isinstance(cmd_payload, dict) else {}
        if not self.fc:
            print(f"⚠️ NAMED CMD {cmd}: no FC connection")
            return cmd in ('LAND', 'TAKEOFF', 'ARM', 'DISARM', 'RTL', 'RTH',
                           'RETURN_TO_USER', 'RTH_USER', 'RETURN_USER',
                           'STABILIZE', 'ALT_HOLD', 'LOITER', 'POSHOLD', 'GUIDED', 'AUTO')

        if cmd == 'LAND':
            alt = self.telemetry_cache.get('altitude', 0)
            print(f"🛬 LAND CMD (local). Alt={alt:.1f}m")
            # RESTORED (wiped): 16s cmd_vel lockout so the laptop stream can't fight the landing,
            # + guaranteed disarm backstop after the descent window.
            self._cmd_lock_until = time.time() + 16.0
            if alt < 1.0:
                print("🛑 GROUND: FORCE DISARM (21196)")
                self.fc.mav.command_long_send(self.fc.target_system, self.fc.target_component,
                                              mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                              0, 0, 21196, 0, 0, 0, 0, 0)
            else:
                self.fc.mav.command_long_send(self.fc.target_system, self.fc.target_component,
                                              mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
                asyncio.create_task(self._smart_avoidance_monitor('land'))
                asyncio.create_task(self._land_backstop_disarm())
            return True

        if cmd == 'TAKEOFF':
            # SITL-FOUND (2026-07-08): NAV_TAKEOFF is only valid in GUIDED — and right after boot
            # the EKF origin can lag the GPS fix by several seconds, so the GUIDED switch itself
            # gets DENIED ("requires position"). Absorb the race here: keep re-requesting GUIDED
            # until the FC actually reports it (EKF ready), THEN command the climb.
            alt = float(p.get('alt', 5))
            for _ in range(24):                                   # up to ~12s of EKF settling
                self.fc.mav.set_mode_send(self.fc.target_system,
                                          mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4)
                await asyncio.sleep(0.5)
                if str(self.telemetry_cache.get('mode_id', '')) in ('GUIDED', '4'):
                    break
            else:
                print("🛫 TAKEOFF: FC never accepted GUIDED (EKF position not ready?) — sending anyway")
            self.fc.mav.command_long_send(self.fc.target_system, self.fc.target_component,
                                          mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)
            return True

        if cmd == 'ARM':
            self.fc.mav.set_mode_send(self.fc.target_system,
                                      mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0)  # STABILIZE
            print("🛡️ SPLIT-ARM (local): Mode -> STABILIZE... then ARM")
            self.fc.mav.command_long_send(self.fc.target_system, self.fc.target_component,
                                          mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                          0, 1, 0, 0, 0, 0, 0, 0)
            return True

        if cmd == 'DISARM':
            # RESTORED (wiped): KILL semantics — 5x force-disarm + 8s cmd_vel lockout so the
            # laptop throttle stream cannot re-spin the motors.
            self._cmd_lock_until = time.time() + 8.0
            for _ in range(5):
                self.fc.mav.command_long_send(self.fc.target_system, self.fc.target_component,
                                              mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                              0, 0, 21196, 0, 0, 0, 0, 0)
            print("LOCAL DISARM: FORCE DISARM x5 + 8s cmd lockout")
            return True

        if cmd in ('RTL', 'RTH'):
            # RESTORED (wiped): GPS gate — RTL without a 3D fix is a BLIND return (the FC doesn't
            # know where home is) -> LAND in place instead.
            _fix = int(self.telemetry_cache.get('gps_fix', 0) or 0)
            if _fix < 3:
                print(f"🏠 RTH: no GPS (fix={_fix}) — LANDING IN PLACE (safe, no blind RTL)")
                self._cmd_lock_until = time.time() + 16.0
                self.fc.mav.set_mode_send(self.fc.target_system,
                                          mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 9)
                asyncio.create_task(self._land_backstop_disarm())
            elif self.batt_rth_destination == 'user' and self.user_gps:
                lat, lng = self.user_gps
                print(f"🏠 RTH TO USER (local): ({lat:.6f}, {lng:.6f})")
                self.fc.mav.set_mode_send(self.fc.target_system,
                                          mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4)
                self.fc.mav.mission_item_int_send(
                    self.fc.target_system, self.fc.target_component,
                    0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 0, 0, 0, 0,
                    int(lat * 1e7), int(lng * 1e7), 15)
            else:
                print("🏠 RTH TO LAUNCH (local RTL)")
                self.fc.set_mode('RTL')
            asyncio.create_task(self._smart_avoidance_monitor('rth'))
            return True

        if cmd in ('RETURN_TO_USER', 'RTH_USER', 'RETURN_USER'):
            # Explicit user return: coords from the payload (laptop sends lat/lng) or live user_gps.
            # GPS gate: navigating to the user needs the DRONE's own 3D fix — without one the goto
            # (and a blind RTL) can't navigate -> LAND in place instead.
            _fix = int(self.telemetry_cache.get('gps_fix', 0) or 0)
            lat = p.get('lat'); lng = p.get('lng')
            if lat is None and self.user_gps:
                lat, lng = self.user_gps
            if _fix < 3:
                print(f"🏠 RETURN TO USER: no GPS (fix={_fix}) — LANDING IN PLACE (safe)")
                self._cmd_lock_until = time.time() + 16.0
                self.fc.mav.set_mode_send(self.fc.target_system,
                                          mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 9)
                asyncio.create_task(self._land_backstop_disarm())
            elif lat is not None:
                alt = float(p.get('alt', 15))
                print(f"🏠 RETURN TO USER (local): ({float(lat):.6f}, {float(lng):.6f}) @ {alt}m")
                self.fc.mav.set_mode_send(self.fc.target_system,
                                          mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4)
                self.fc.mav.mission_item_int_send(
                    self.fc.target_system, self.fc.target_component,
                    0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 0, 0, 0, 0,
                    int(float(lat) * 1e7), int(float(lng) * 1e7), alt)
                asyncio.create_task(self._smart_avoidance_monitor('rth'))
            else:
                print("🏠 RETURN TO USER: no user location — RTL (drone has GPS fix)")
                self.fc.set_mode('RTL')
            return True

        if cmd in ('STABILIZE', 'ALT_HOLD', 'LOITER', 'POSHOLD', 'GUIDED', 'AUTO'):
            mode_map = {'STABILIZE': 0, 'ALT_HOLD': 2, 'AUTO': 3,
                        'GUIDED': 4, 'LOITER': 5, 'POSHOLD': 16}
            print(f"🛩️ MODE (local): {cmd}")
            self.fc.mav.set_mode_send(self.fc.target_system,
                                      mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_map[cmd])
            return True

        if cmd == 'SET_SPEED':
            # Cinematic speed change (m/s) -> the FC's real WPNAV_SPEED param (cm/s). Was previously
            # UNHANDLED anywhere (silent no-op). The FC echoes PARAM_VALUE -> fc_caps -> the laptop's
            # movement envelope updates automatically, so the AI's speed ceiling follows it live.
            try:
                ms = float(p.get('value', 5.0))
                cms = max(50, min(2000, int(ms * 100)))
                print(f"🎬 SET_SPEED (local): {ms} m/s -> WPNAV_SPEED={cms}")
                self.fc.mav.param_set_send(self.fc.target_system, self.fc.target_component,
                                           b'WPNAV_SPEED', float(cms),
                                           mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            except Exception as e:
                print(f"⚠️ SET_SPEED failed: {e}")
            return True

        return False   # not a named command -> caller falls through to execute_ai_plan

    def _begin_route_mission(self, items, autostart=True, alt=None):
        """Upload a route mission to the FC using the proper MISSION_REQUEST/ACK handshake.

        Single implementation shared by every mission entry point. Two details here are
        load-bearing and were established by SITL testing:

        1. No mission_clear_all beforehand. Its own MISSION_ACK arrives first and is
           indistinguishable from the upload-complete ACK, which produced 'Mission upload
           timeout' followed by AUTO 'init failed'. A new mission_count transaction
           replaces the previous mission on its own.
        2. Items are not blind-blasted. ArduPilot pulls each item by sequence number; the
           read loop answers MISSION_REQUEST from self._pending_mission.

        The uploaded list is always [home placeholder, TAKEOFF, *waypoints], so the read
        loop's waypoint count (len - 2) stays correct for every caller.

        Returns the number of route waypoints queued, or 0 if there was nothing to send.
        """
        items = [it for it in (items or [])
                 if isinstance(it, dict) and 'lat' in it and 'lng' in it]
        if not items or not self.fc:
            return 0

        if alt is None:
            alt = float(os.getenv('MISSION_ALT_M', '15'))
        alt = float(alt)
        first = items[0]

        pm = [{'cmd': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,   # seq0 = home placeholder
               'lat': first['lat'], 'lng': first['lng'], 'alt': 0},
              {'cmd': mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,    # seq1 = auto-takeoff
               'lat': first['lat'], 'lng': first['lng'], 'alt': alt}]
        pm += [{'cmd': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                'lat': it['lat'], 'lng': it['lng'],
                'alt': float(it.get('alt', alt))} for it in items]

        self._pending_mission = pm
        self._mission_autostart = bool(autostart)
        self.fc.mav.mission_count_send(
            self.fc.target_system, self.fc.target_component, len(pm))
        print(f"🗺️ ROUTE MISSION: {len(items)} waypoints @ {alt}m "
              f"(+takeoff item) — handshake upload started")
        return len(items)

    async def process_packet(self, type, payload):
        """Process a command from local WebSocket client (Tailscale/laptop AI).
           Routes into the same logic as command_loop but without needing cloud WS."""
        if not type:
            return

        # user_gps from app via Tailscale
        if type == 'user_gps':
            if payload and isinstance(payload, dict) and 'lat' in payload:
                self.user_gps = (payload.get('lat'), payload.get('lng'))
            return

        # joystick from app via Tailscale
        if type == 'joystick':
            if not isinstance(payload, dict):
                return
            try:
                def map_ch(val, center=True):
                    if center: return int(1500 + (val * 500))
                    return int(1000 + (val * 1000))

                raw_roll  = float(payload.get('x', 0))
                raw_pitch = float(payload.get('y', 0))
                raw_thr   = float(payload.get('z', 0))
                raw_yaw   = float(payload.get('r', 0))

                rc1 = map_ch(raw_roll)
                rc2 = map_ch(raw_pitch)
                rc3 = map_ch(raw_thr)
                rc4 = map_ch(raw_yaw)

                if self.fc:
                    self.fc.mav.rc_channels_override_send(
                        self.fc.target_system, self.fc.target_component,
                        rc1, rc2, rc3, rc4, 0, 0, 0, 0
                    )
            except Exception as e:
                print(f"⚠️ LOCAL joystick error: {e}")
            return

        # gimbal from laptop AI via Tailscale
        if type == 'gimbal':
            if isinstance(payload, dict):
                try:
                    self.esp32_cmd_queue.put_nowait({
                        "type": "gimbal",
                        "pitch": payload.get('pitch', 0),
                        "yaw": payload.get('yaw', 0)
                    })
                except: pass
            return

        # AI velocity commands from laptop AI.
        # RESTORED (wiped 6-30..7-02): the laptop's AutopilotController sends type='cmd_vel' (NOT
        # 'velocity') — with only 'velocity' accepted, the ENTIRE autonomous-flight velocity stream
        # was silently ignored on the local link. Accept BOTH, honour the LAND/DISARM kill-lockout,
        # and route through execute_ai_plan so the no-GPS ALT_HOLD RC-override conversion (restored
        # there) applies — that conversion is what made the 6-24 indoor flight work.
        if type in ('velocity', 'cmd_vel'):
            if time.time() < getattr(self, '_cmd_lock_until', 0):
                return   # LAND/DISARM kill-lockout: ignore the laptop throttle stream
            if isinstance(payload, dict) and self.fc:
                try:
                    await self.execute_ai_plan({'action': 'CMD_VEL', 'params': {
                        'vx': float(payload.get('vx', 0)),
                        'vy': float(payload.get('vy', 0)),
                        'vz': float(payload.get('vz', 0)),
                        'yaw_rate': float(payload.get('yaw_rate', 0)),
                    }})
                except Exception as e:
                    print(f"⚠️ LOCAL velocity error: {e}")
            return

        # AI plan from laptop AI
        if type == 'ai_plan' or type == 'AI_PLAN':
            if isinstance(payload, dict):
                await self.execute_ai_plan(payload)
            return

        # Generic commands (LAND, ARM, RTH, etc.)
        if type == 'command':
            cmd = payload if isinstance(payload, str) else (payload.get('command', '') if isinstance(payload, dict) else '')
            cmd_payload = payload.get('payload', {}) if isinstance(payload, dict) and not isinstance(payload, str) else {}
            if isinstance(cmd, str) and cmd:
                # Upper-case the COMMAND TOKEN only. 'SET_CONFIG:rth_behavior=user' must keep
                # its value case: a blanket .upper() turned it into '...=USER' and '4k' into
                # '4K', so every value the app sent arrived corrupted.
                cmd = cmd.strip()
                if cmd.startswith('SET_CONFIG:') or cmd.upper().startswith('SET_CONFIG:'):
                    cmd = 'SET_CONFIG:' + cmd.split(':', 1)[1]
                else:
                    cmd = cmd.upper()
                # Safety check
                is_emergency = (cmd in ['LAND', 'DISARM', 'RTL', 'UPDATE_CONFIG']
                                or cmd.startswith('SET_'))  # settings never move the aircraft
                if not is_emergency and not self.safety.validate_auto_action(cmd):
                    print(f"🛑 SAFETY BLOCK (local): {cmd}")
                    return
                # NAMED flight commands (LAND/RTL/ARM/mode names/...) get REAL execution here.
                # Previously they ALL fell into execute_ai_plan, which only understands numeric
                # setpoints (vx/vy, lat/lng, mode key) -> every named command over the LOCAL link
                # (laptop AI + app on Tailscale) was a SILENT NO-OP (only the cloud path executed them).
                if await self._execute_named_command(cmd, cmd_payload):
                    return
                # Operator settings (SET_CONFIG / thresholds / camera). These used to exist only
                # in the cloud command_loop, so over the local link they silently became bogus
                # 'AI PLAN' actions -- the app's return destination and battery threshold never
                # reached the failsafe.
                if await self._execute_settings_command(cmd, cmd_payload):
                    return
                # Not a named command -> AI plan executor (numeric setpoints) as before
                await self.execute_ai_plan({'action': cmd, **cmd_payload})
            return

        # Director intent from laptop AI
        # APP ROUTE MISSION over the LOCAL link (was: silently dropped as 'unknown type'!).
        # The app sends {"type":"mission","payload":[{lat,lng},...]} with NO start command and NO
        # altitudes — so: prepend home+TAKEOFF items, upload via the proper MISSION_REQUEST/ACK
        # handshake (read-loop answers each seq), and auto-start AUTO on ACK if already armed.
        if type == 'mission':
            items = payload if isinstance(payload, list) else (payload or {}).get('items', [])
            self._begin_route_mission(items, autostart=True)
            return

        if type == 'director_intent':
            print(f"🎬 LOCAL Director Intent: {str(payload)[:80]}")
            # Forward to cloud if connected
            if self.ws:
                try:
                    await self.ws.send(json.dumps({"type": "director_intent", "payload": payload}))
                except: pass
            return

        # Fallback: try to route as command
        print(f"⚠️ LOCAL unknown type: {type}")

    async def command_loop(self):
        while self.running and self.ws:
            try:
                msg = await self.ws.recv()
                self.last_cloud_msg = time.time()  # Update watchdog timer
                # V57: RAW DEBUG (See exact JSON)
                if 'joystick' not in msg:
                     print(f"🔍 RAW JSON: {msg}")

                data = json.loads(msg)
                type = data.get('type')
                # Support both 'payload' (app/cloud) and 'primitive' (laptop AI)
                payload = data.get('payload', data.get('primitive', {}))

                # V58 FIX: Double-Decode only if JSON-String. Keep original if fail.
                if isinstance(payload, str):
                    try:
                        decoded = json.loads(payload)
                        if isinstance(decoded, (dict, list)): # Only accept proper structures
                             payload = decoded
                    except:
                        pass # It was a plain string (e.g. "LAND") - Keep it!
                if type != 'joystick' and type != 'user_gps': # Spam filter: Hide GPS too
                    print(f"📥 Rx: {type}")

                if type == 'user_gps':
                    if payload and 'lat' in payload:
                        self.user_gps = (payload.get('lat'), payload.get('lng'))
                # V55: OMNI-PARSER
                cmd = type

                # V110: APP COMMAND FORMAT FIX
                # App sends {"type": "command", "payload": "LAND"} via sendCommand()
                # We need to extract the real command from payload
                if cmd == 'command' and isinstance(payload, str):
                    cmd = payload  # "LAND", "ARM", "RTH", etc.
                    payload = {}   # Reset payload since it was the command string
                elif cmd == 'command' and isinstance(payload, dict) and 'command' in payload:
                    # App sendCommand("set_safety_config", {...}) sends {"type":"command","payload":{"command":"set_safety_config","payload":{...}}}
                    cmd = payload.get('command', cmd)
                    payload = payload.get('payload', payload)

                # Also handle "mission" type from app's sendMission()
                if cmd == 'mission':
                    cmd = 'UPLOAD_MISSION'
                    payload = {'items': payload if isinstance(payload, list) else payload.get('items', [])}

                # V56: STRING CLEANUP
                if isinstance(cmd, str):
                    cmd = cmd.upper().strip().replace('"', '').replace("'", "")

                # V56: Safety Check
                is_emergency = cmd in ['LAND', 'DISARM', 'RTL', 'UPDATE_CONFIG']
                if not is_emergency and not self.safety.validate_auto_action(cmd):
                     print(f"🛑 SAFETY BLOCK: {cmd}")
                     if self.ws:
                         await self.ws.send(json.dumps({
                             "type": "safety_block",
                             "payload": {
                                 "blocked_action": cmd,
                                 "safety": self.safety.get_safety_status()
                             }
                         }))
                     continue  # ENFORCED: Block unsafe actions

                # V110: Handle SET_CONFIG:key=value format from app
                if cmd.startswith('SET_CONFIG:'):
                    # Parse "SET_CONFIG:res=4k" → key="res", value="4k"
                    config_str = cmd.replace('SET_CONFIG:', '')
                    if '=' in config_str:
                        key, val = config_str.split('=', 1)
                        print(f"⚙️ SET_CONFIG: {key}={val}")
                        # Forward as UPDATE_CONFIG
                        payload = {'config': {key.strip().lower(): val.strip()}}
                        cmd = 'UPDATE_CONFIG'
                    else:
                        print(f"⚠️ Malformed SET_CONFIG: {config_str}")

                # V110: Handle set_safety_config from app
                elif cmd == 'SET_SAFETY_CONFIG':
                    if isinstance(payload, dict):
                        oa = payload.get('obstacle_avoidance', True)
                        vp = payload.get('vision_pos', True)
                        print(f"⚙️ SAFETY CONFIG: obstacle_avoidance={oa}, vision_pos={vp}")
                        # Store in telemetry cache for reference
                        self.telemetry_cache['obstacle_avoidance'] = oa
                        self.telemetry_cache['vision_pos'] = vp

                # V110: Handle set_rth_alt from app
                elif cmd == 'SET_RTH_ALT':
                    alt_cm = int(payload.get('alt', 5000)) if isinstance(payload, dict) else 5000
                    alt_m = alt_cm / 100.0
                    print(f"⚙️ RTH ALT: {alt_m}m")
                    # Set RTL_ALT param on FC (in cm)
                    if self.fc:
                        self.fc.mav.param_set_send(
                            self.fc.target_system, self.fc.target_component,
                            b'RTL_ALT', alt_cm, mavutil.mavlink.MAV_PARAM_TYPE_INT32
                        )

                # V110: Handle set_batt_threshold from app
                elif cmd == 'SET_BATT_THRESHOLD':
                    threshold = int(payload.get('threshold', 20)) if isinstance(payload, dict) else 20
                    self.batt_threshold = max(5, min(50, threshold))
                    print(f"⚙️ BATT THRESHOLD: {self.batt_threshold}%")

                # V110: SWITCH_CAMERA — user picks which camera to view/record
                elif cmd == 'SWITCH_CAMERA':
                    target = payload.get('camera', 'internal') if isinstance(payload, dict) else 'internal'
                    if target in ('internal', 'external', 'gopro'):
                        self.active_cam = 'external' if target in ('external', 'gopro') else 'internal'
                        print(f"📷 CAMERA SWITCH: active={self.active_cam}")
                        self.telemetry_cache['active_cam'] = self.active_cam

                elif cmd == 'SET_RECORDING_CAMERA':
                    target = payload.get('camera', 'internal') if isinstance(payload, dict) else 'internal'
                    self.recording_cam = 'external' if target in ('external', 'gopro') else 'internal'
                    print(f"🎬 RECORDING CAMERA: {self.recording_cam}")

                # P2.1: SETTINGS SYNC
                if cmd == 'UPDATE_CONFIG':
                    cfg = payload.get('config', {})
                    print(f"⚙️ SETTINGS UPDATED: {cfg}")  # V120_SETTINGS_WIRED (restored — was wiped)
                    # Connect the app's Return/Land/Battery settings to the actual return logic.
                    if 'rth_behavior' in cfg:
                        self.batt_rth_destination = 'user' if str(cfg['rth_behavior']).lower() == 'user' else 'launch'
                        print(f"⚙️ RTH DEST -> {self.batt_rth_destination}")
                    if 'land_behavior' in cfg:
                        self.land_destination = 'user' if str(cfg['land_behavior']).lower() == 'user' else 'here'
                        print(f"⚙️ LAND DEST -> {self.land_destination}")
                    if 'batt_threshold' in cfg:
                        try:
                            self.batt_threshold = max(5, min(50, int(float(cfg['batt_threshold']))))
                            self.low_batt_triggered = False
                            print(f"⚙️ BATT THRESHOLD -> {self.batt_threshold}%")
                        except Exception:
                            pass

                    # HANDLE CAMERA RESOLUTION CHANGE
                    if 'cap_res' in cfg:
                        res_key = cfg['cap_res']
                        # IMX219 (Pi Cam V2) Modes
                        RES_MAP = {
                            "8mp": "video/x-raw,width=3280,height=2464,framerate=15/1",
                            "1080p": "video/x-raw,width=1920,height=1080,framerate=30/1",
                            "720p": "video/x-raw,width=1280,height=720,framerate=60/1",
                            "480p": "video/x-raw,width=640,height=480,framerate=90/1"
                        }
                        # If unknown (e.g. GoPro res), ignore it
                        if res_key in RES_MAP:
                            new_caps = RES_MAP[res_key]
                            # Only reset if changed
                            if new_caps != getattr(self, 'target_caps', ""):
                                self.target_caps = new_caps
                                self.cam_needs_reset = True
                                print(f"🎥 CAMERA CONFIG CHANGING TO: {res_key} ({new_caps})")

                # P2.8: GOPRO REMOTE AI PROXY
                elif cmd == 'GOPRO_SETTINGS':
                    if self.gopro_proxy:
                        print(f"🔵 PROXY: Forwarding AI Cinematic Params to GoPro BLE: {payload}")
                        await self.gopro_proxy.handle_remote_ai_command(payload)
                    else:
                        print("⚠️ Received GoPro AI Params, but Local Proxy is offline.")

                # AI AUTONOMOUS PLAN HANDLER
                # The AI has FULL authority to decide any action.
                # This converts the AI's decision into real MAVLink commands.
                elif cmd == 'AI_PLAN':
                    await self.execute_ai_plan(payload)

                # GENERIC COMMAND RELAY (mavllink_executor.py sends these)
                elif cmd == 'COMMAND':
                    await self.execute_ai_plan({
                        'action': payload.get('action', 'RELAY'),
                        'params': payload.get('params', payload),
                    })

                # LAPTOP AI VELOCITY RELAY (remote mode sends cmd_vel)
                elif cmd == 'CMD_VEL':
                    await self.execute_ai_plan({
                        'action': 'CMD_VEL',
                        'params': {
                            'vx': payload.get('vx', 0),
                            'vy': payload.get('vy', 0),
                            'vz': payload.get('vz', 0),
                            'yaw_rate': payload.get('yaw_rate', 0),
                        }
                    })

                # P2.2: GIMBAL RELAY
                elif cmd == 'GIMBAL':
                    pitch = payload.get('pitch', 0)
                    yaw = payload.get('yaw', 0)
                    await self.esp32_cmd_queue.put({"type": "gimbal", "pitch": pitch, "yaw": yaw})
                    print(f"🎥 GIMBAL: Pitch={pitch}, Yaw={yaw}")

                    # P1.6: MISSION HANDLER (Upload Waypoints to FC)
                elif cmd == 'UPLOAD_MISSION':
                    # Routed through the same handshake uploader as the local path. The former
                    # inline implementation cleared the mission first and blind-blasted items
                    # without waiting for MISSION_REQUEST, which SITL showed fails the upload.
                    try:
                        items = payload.get('items', []) if isinstance(payload, dict) else payload
                        if not self._begin_route_mission(items, autostart=False):
                            print("⚠️ UPLOAD_MISSION: no valid waypoints in payload")
                    except Exception as e:
                        print(f"❌ Mission Upload Fail: {e}")

                elif cmd == 'TAKEOFF':
                    self.fc.mav.command_long_send(self.fc.target_system, self.fc.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 5)

                elif cmd == 'LAND':
                    alt = self.telemetry_cache.get('altitude', 0)
                    print(f"🛬 LAND CMD. Alt={alt:.1f}m")
                    if alt < 1.0:
                        print("🛑 GROUND: FORCE DISARM (21196)")
                        self.fc.mav.command_long_send(self.fc.target_system, self.fc.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196, 0, 0, 0, 0, 0)
                    else:
                        print("🛬 AIR: SMART LAND (with obstacle avoidance)")
                        self.fc.mav.command_long_send(self.fc.target_system, self.fc.target_component, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
                        # V110: Start smart avoidance monitor during landing
                        asyncio.create_task(self._smart_avoidance_monitor('land'))

                elif cmd == 'ARM':
                     self.fc.mav.set_mode_send(self.fc.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0) # STABILIZE
                     print("🛡️ SPLIT-ARM: Mode -> STABILIZE (0)... NO DELAY")
                     self.fc.mav.command_long_send(self.fc.target_system, self.fc.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
                     print("🛡️ SPLIT-ARM: Sending ARM Command Now.")

                elif cmd in ('RTL', 'RTH'):
                    # V110: RTH from app = RTL to ArduPilot
                    # If user_gps available and rth_behavior is 'user', fly to user instead
                    if self.batt_rth_destination == 'user' and self.user_gps:
                        lat, lng = self.user_gps
                        print(f"🏠 RTH TO USER: ({lat:.6f}, {lng:.6f})")
                        self.fc.mav.set_mode_send(self.fc.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4)
                        self.fc.mav.mission_item_int_send(
                            self.fc.target_system, self.fc.target_component,
                            0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 0, 0, 0, 0,
                            int(lat * 1e7), int(lng * 1e7), 15
                        )
                    else:
                        print("🏠 RTH TO LAUNCH (RTL)")
                        self.fc.set_mode('RTL')
                    # V110: Start smart avoidance monitor during RTH
                    asyncio.create_task(self._smart_avoidance_monitor('rth'))

                elif cmd == 'DISARM':
                    print("DISARM command received")
                    self.fc.mav.command_long_send(
                        self.fc.target_system, self.fc.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                        0, 0, 21196, 0, 0, 0, 0, 0  # 21196 = force disarm
                    )

                # RETURN TO USER — fly to user's live GPS position
                elif cmd in ('RETURN_TO_USER', 'RTH_USER', 'RETURN_USER'):
                    if self.user_gps:
                        lat, lng = self.user_gps
                        alt = float(payload.get('alt', 15)) if isinstance(payload, dict) else 15
                        print(f"RTH TO USER: ({lat}, {lng}) @ {alt}m")
                        self.fc.mav.set_mode_send(
                            self.fc.target_system,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4  # GUIDED
                        )
                        self.fc.mav.mission_item_int_send(
                            self.fc.target_system, self.fc.target_component,
                            0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                            2, 0, 0, 0, 0, 0,
                            int(lat * 1e7), int(lng * 1e7), alt
                        )
                    else:
                        print("RTH USER: No user GPS — falling back to RTL")
                        self.fc.set_mode('RTL')

                # V110: AI AVOIDANCE RESPONSE — from cloud/laptop AI
                elif cmd == 'AI_AVOIDANCE':
                    # AI sends {vx, vy, vz} avoidance velocity vector
                    if isinstance(payload, dict):
                        self.telemetry_cache['ai_avoidance_vector'] = {
                            'vx': float(payload.get('vx', 0)),
                            'vy': float(payload.get('vy', 0)),
                            'vz': float(payload.get('vz', 0)),
                        }
                        print(f"🤖 AI AVOIDANCE RECEIVED: vx={payload.get('vx',0)} vy={payload.get('vy',0)} vz={payload.get('vz',0)}")

                # FOLLOW ME — continuously fly toward user's phone GPS
                elif cmd == 'FOLLOW_ME':
                    enabled = True
                    if isinstance(payload, dict):
                        enabled = payload.get('enabled', True)
                    self.follow_me_active = enabled
                    if enabled:
                        print("FOLLOW ME: ACTIVE — tracking user GPS")
                    else:
                        print("FOLLOW ME: STOPPED")
                        self.fc.mav.set_position_target_local_ned_send(
                            0, self.fc.target_system, self.fc.target_component,
                            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                            0b0000111111000111,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                        )

                # BATTERY THRESHOLD — set battery % for auto-RTH
                elif cmd == 'SET_BATT_RTH':
                    pct = int(payload.get('percent', 20)) if isinstance(payload, dict) else 20
                    self.batt_threshold = max(5, min(50, pct))
                    # User can choose RTH destination
                    dest = payload.get('destination', 'launch') if isinstance(payload, dict) else 'launch'
                    self.batt_rth_destination = dest  # 'launch' or 'user'
                    print(f"BATT RTH: {self.batt_threshold}% -> {dest}")

                # MODE SWITCH — app sends mode name directly
                elif cmd in ('STABILIZE', 'ALT_HOLD', 'LOITER', 'POSHOLD', 'GUIDED', 'AUTO'):
                    mode_map = {
                        'STABILIZE': 0, 'ALT_HOLD': 2, 'LOITER': 5,
                        'POSHOLD': 16, 'GUIDED': 4, 'AUTO': 3,
                    }
                    mode_id = mode_map.get(cmd, 0)
                    print(f"MODE SWITCH: {cmd} ({mode_id})")
                    self.fc.mav.set_mode_send(
                        self.fc.target_system,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                        mode_id
                    )

                # P2.5: CAPTURE/RECORD HANDLERS - Camera control (INTERCEPTED FOR RADXA)
                # V110: Support both CAPTURE and CAPTURE_PHOTO from app
                elif cmd in ('CAPTURE', 'CAPTURE_PHOTO') or type == 'capture':
                    print("📸 CAPTURE: Triggering Local Save...")
                    self.take_photo_flag = True
                    await self.esp32_cmd_queue.put({"type": "capture"})

                # V110: Support START_RECORDING / STOP_RECORDING from app
                elif cmd in ('RECORD', 'START_RECORDING', 'STOP_RECORDING') or type == 'record':
                    # V110: Robust recording state detection
                    recording = False
                    if cmd == 'START_RECORDING':
                        recording = True
                    elif cmd == 'STOP_RECORDING':
                        recording = False
                    elif isinstance(payload, dict):
                        recording = payload.get('recording', False)
                    elif isinstance(payload, bool):
                        recording = payload

                    print(f"🎥 RECORD: {'START' if recording else 'STOP'} (Local)")
                    self.is_recording = recording
                    # Also send to ESP32 for feedback (e.g. solid RED LED)
                    await self.esp32_cmd_queue.put({"type": "record", "recording": recording})
                elif type == 'joystick':
                    try:
                        # Extract and Clamping
                        def map_ch(val, center=True):
                            if center: return int(1500 + (val * 500))
                            return int(1000 + (val * 1000)) # 0-1 range to 1000-2000
                        # V110: CORRECT MODE 2 MAPPING (Standard RC Controller)
                        # App Left Stick:  z = throttle (up/down), r = yaw (left/right)
                        # App Right Stick: x = roll (left/right),  y = pitch (up/down)
                        # ArduPilot RC channels: RC1=Roll, RC2=Pitch, RC3=Throttle, RC4=Yaw

                        is_armed = self.fc.motors_armed() if self.fc else False

                        # Apply expo curve for RC feel (0.0 = linear, 1.0 = max expo)
                        expo = float(self.telemetry_cache.get('expo', 0.3))
                        def apply_expo(val, exp):
                            """Cubic expo curve: output = (1-exp)*val + exp*val^3"""
                            sign = 1 if val >= 0 else -1
                            abs_val = abs(val)
                            return sign * ((1.0 - exp) * abs_val + exp * (abs_val ** 3))

                        # Deadzone: ignore tiny stick movements (<5%)
                        def deadzone(val, dz=0.05):
                            if abs(val) < dz: return 0.0
                            sign = 1 if val >= 0 else -1
                            return sign * (abs(val) - dz) / (1.0 - dz)

                        # Extract raw values from payload
                        raw_roll  = deadzone(float(payload.get('x', 0)))   # Right stick X → Roll
                        raw_pitch = deadzone(float(payload.get('y', 0)))   # Right stick Y → Pitch
                        raw_thr   = deadzone(float(payload.get('z', 0)))   # Left stick Y → Throttle
                        raw_yaw   = deadzone(float(payload.get('r', 0)))   # Left stick X → Yaw

                        # Apply expo for RC feel
                        raw_roll  = apply_expo(raw_roll, expo)
                        raw_pitch = apply_expo(raw_pitch, expo)
                        raw_yaw   = apply_expo(raw_yaw, expo)
                        # Throttle: no expo (linear is better for throttle control)

                        # Map to RC PWM (1000-2000, center 1500)
                        target_rc1 = map_ch(raw_roll, center=True)    # RC1 = Roll
                        target_rc2 = map_ch(raw_pitch, center=True)   # RC2 = Pitch
                        target_rc3 = map_ch(raw_thr, center=True)     # RC3 = Throttle
                        target_rc4 = map_ch(raw_yaw, center=True)     # RC4 = Yaw

                        # V50: REMOVE SMOOTHING ENTIRELY (Lag Fix Check)
                        # Direct mapping. No buffer.
                        rc4 = target_rc4
                        rc2 = target_rc2
                        rc1 = target_rc1

                        # V64: ADAPTIVE THROTTLE CURVE
                        # Use MOT_THST_HOVER (learned by AltHold) as the Center Stick Target

                        hover_param = 0.55 # F450 Heavy Default (1.5kg)
                        try:
                             # Try to read cached value (updated by watchdog)
                             hover_param = self.telemetry_cache.get('MOT_THST_HOVER', 0.55)
                             if hover_param < 0.1 or hover_param > 0.8: hover_param = 0.55 # Sanity Check
                        except:
                             hover_param = 0.55

                        hover_pwm = 1000 + (hover_param * 1000) # e.g. 0.55 -> 1550

                        t_in = target_rc3
                        t_out = 1000

                        if t_in < 1500:
                            # Low Half: Map 1000-1500 -> 1100-HoverPWM
                            pct = (t_in - 1000) / 500.0
                            t_out = 1100 + (pct * (hover_pwm - 1100))
                        else:
                            # High Half: Map 1500-2000 -> HoverPWM-2000 (V66: FULL POWER)
                            pct = (t_in - 1500) / 500.0
                            t_out = hover_pwm + (pct * (2000 - hover_pwm))

                        rc3 = int(t_out)

                        # V110: TILT COMPENSATION (Gentle)
                        # When tilting forward/sideways, add gentle throttle to maintain altitude
                        # Max boost: +200 PWM (was 800 — way too aggressive)
                        tilt_pitch = abs(target_rc2 - 1500)
                        tilt_roll = abs(target_rc1 - 1500)
                        max_tilt = max(tilt_pitch, tilt_roll)  # 0 to 500

                        if max_tilt > 100:  # Only compensate for significant tilt
                            boost = (max_tilt / 500.0) * 200.0  # Max +200 PWM
                            rc3 += int(boost)
                            if rc3 > 2000: rc3 = 2000

                        # V50: MOTOR SAFETY (NO STOPPING IN AIR) - User wants LINEAR DESCENT
                        # (Curve starts at 1100. We just clamp min to 1100 to prevent disarm in air)
                        if is_armed:
                             if rc3 < 1100: rc3 = 1100 # Safety Floor (Idle Only)

                        if abs(target_rc2 - 1500) > 50 or abs(target_rc3 - 1500) > 50:
                             # V65 DEBUG: Show what Pitch/Roll we are actually sending
                             print(f"🕹️ MIX: Pitch(RC2)={rc2} Roll(RC1)={rc1} Thr(RC3)={rc3}")
                        # V34: TOY MODE (Auto-Arm on Throttle) BEFORE CLAMPING CHECK
                        # (Logic handled by 'is_armed' check above)

                        # Auto-arm: only if NOT armed AND not already attempted recently
                        # BENCH SAFETY: ALLOW_AUTO_ARM gate prevents any auto-arm (props can't spin)
                        if ALLOW_AUTO_ARM and not is_armed and not getattr(self, '_auto_armed_done', False):
                            if rc3 > 1400:
                                 self.auto_arm_counter = getattr(self, 'auto_arm_counter', 0) + 1
                                 real_rc3 = rc3
                                 rc3 = 1000  # Zero throttle for arming

                                 if self.auto_arm_counter > 10:
                                      gps_sats = self.telemetry_cache.get('satellites', 0)
                                      if gps_sats >= 6:
                                          self.fc.mav.set_mode_send(self.fc.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 5)
                                          print("🚀 AUTO-ARM: LOITER mode (GPS)")
                                      else:
                                          self.fc.mav.set_mode_send(self.fc.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 2)
                                          print("🚀 AUTO-ARM: ALT_HOLD mode")
                                      self.fc.mav.command_long_send(self.fc.target_system, self.fc.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
                                      print("🚀 AUTO-ARM: Armed!")
                                      self.auto_arm_counter = 0
                                      self._auto_armed_done = True  # Don't re-arm again
                            else:
                                 self.auto_arm_counter = 0
                        elif is_armed:
                            self._auto_armed_done = True  # Already armed (by manual command)
                        self.fc.mav.rc_channels_override_send(
                            self.fc.target_system, self.fc.target_component,
                            rc1, rc2, rc3, rc4, 65535, 65535, 65535, 65535
                        )

                        # V31: STICK ARMING LOGIC (Backup - Down-Right)
                        # BENCH SAFETY: gated behind ALLOW_AUTO_ARM (props can't spin)
                        if ALLOW_AUTO_ARM and rc3 < 1150 and rc4 > 1900:
                             self.stick_arm_counter = getattr(self, 'stick_arm_counter', 0) + 1
                             if self.stick_arm_counter > 20: # ~2 seconds @ 10Hz
                                  print("🕹️ STICK ARM TRIGGERED!")
                                  self.fc.mav.command_long_send(self.fc.target_system, self.fc.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
                                  self.stick_arm_counter = 0
                        else:
                             self.stick_arm_counter = 0
                        # V15: Debug RC Output
                        # V15: Debug RC Output
                        if int(time.time() * 10) % 20 == 0: # Every 2s
                           print(f"🕹️ JOY: R{rc1} P{rc2} T{rc3} Y{rc4}")

                    except Exception as e:
                         print(f"Joystick Error: {e}")
            except Exception as e:
                print(f"CMD Loop Error: {e}")
                break
    async def _try_connect_gopro(self):
        """V110: Try connecting to GoPro via WiFi UDP or USB"""
        import cv2
        import subprocess

        # Try 0: MediaMTX re-served GoPro feed (rtsp://127.0.0.1:8554/gopro) — the working path for the app /snapshot
        try:
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp"
            cap = cv2.VideoCapture("rtsp://127.0.0.1:8554/gopro", cv2.CAP_FFMPEG)
            if cap.isOpened():
                ret, f = cap.read()
                if ret and f is not None:
                    self.external_cap = cap
                    self.external_available = True
                    self.telemetry_cache['external_available'] = True
                    print(f"✅ EXTERNAL (GoPro via MediaMTX): ACTIVE @ {f.shape[1]}x{f.shape[0]}")
                    return True
                cap.release()
        except Exception as e:
            print(f"⚠️ MediaMTX RTSP capture failed: {e}")

        # Try 1: GoPro USB webcam
        for idx in range(10):
            dev = f"/dev/video{idx}"
            if not os.path.exists(dev): continue
            try:
                result = subprocess.run(['v4l2-ctl', '-d', dev, '--info'],
                    capture_output=True, text=True, timeout=3)
                if 'gopro' in result.stdout.lower() or 'webcam' in result.stdout.lower():
                    pipeline = (f"v4l2src device={dev} ! videoscale ! "
                        "video/x-raw,width=1280,height=720 ! videoconvert ! "
                        "video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false")
                    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
                    if cap.isOpened():
                        ret, f = cap.read()
                        if ret and f is not None:
                            self.external_cap = cap
                            self.external_available = True
                            self.telemetry_cache['external_available'] = True
                            print(f"✅ EXTERNAL (GoPro USB): ACTIVE @ {dev}")
                            return True
                        cap.release()
            except: pass

        # GoPro WiFi UDP: only try if WiFi is already on GoPro network
        # (prevents "bind failed" spam when not connected to GoPro WiFi)
        if getattr(self, '_gopro_wifi_active', False):
            try:
                cap = cv2.VideoCapture("udp://@0.0.0.0:8554", cv2.CAP_FFMPEG)
                if cap.isOpened():
                    ret, f = cap.read()
                    if ret and f is not None:
                        self.external_cap = cap
                        self.external_available = True
                        self.telemetry_cache['external_available'] = True
                        print(f"✅ EXTERNAL (GoPro WiFi UDP): ACTIVE @ {f.shape[1]}x{f.shape[0]}")
                        return True
                    cap.release()
            except: pass

        self.external_available = False
        self.telemetry_cache['external_available'] = False
        print("⚠️ EXTERNAL (GoPro): Not detected")
        return False

    async def _smart_avoidance_monitor(self, mode='rth'):
        """
        V110: Smart Obstacle Avoidance Monitor
        Runs during RTH/Land. Continuously checks sensors.

        PRIORITY: SENSORS > AI
        - If any sensor detects obstacle < CRITICAL_DIST: IMMEDIATE HALT (no AI wait)
        - If obstacle < CAUTION_DIST: Request AI for avoidance path, pause movement
        - If obstacle < WARN_DIST: Log warning, let AI plan ahead

        Sends sensor data + video frame to cloud AI/laptop AI for path planning.
        AI returns avoidance vector which gets applied as GUIDED waypoint offset.
        """
        print(f"🛡️ SMART AVOIDANCE MONITOR STARTED (mode={mode})")
        avoidance_active = False

        while self.running and self.is_armed:
            safety = self.safety.get_safety_status()
            closest = safety['closest_obstacle_m']
            level = safety['level']

            # --- SENSOR PRIORITY: Immediate halt if too close ---
            if level == 'CRITICAL':
                if not avoidance_active:
                    print(f"🛑 AVOIDANCE: HALT! Obstacle @ {closest:.2f}m ({safety['obstacle_source']})")
                    avoidance_active = True
                    # Switch to LOITER to stop all movement
                    self.fc.mav.set_mode_send(
                        self.fc.target_system,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 5)  # LOITER

                    # Send sensor data to AI for avoidance planning
                    if self.ws:
                        try:
                            await self.ws.send(json.dumps({
                                'type': 'avoidance_request',
                                'safety': safety,
                                'tof': {
                                    'front': self.telemetry_cache.get('t1', -1),
                                    'right': self.telemetry_cache.get('t2', -1),
                                    'back': self.telemetry_cache.get('t3', -1),
                                    'left': self.telemetry_cache.get('t4', -1),
                                },
                                'lidar_dist': self.telemetry_cache.get('lidar_dist', 9.9),
                                'altitude': self.telemetry_cache.get('altitude', 0),
                                'heading': self.telemetry_cache.get('heading', 0),
                                'mode': mode,
                                'lat': self.telemetry_cache.get('lat', 0),
                                'lng': self.telemetry_cache.get('lng', 0),
                            }))
                        except: pass

            elif level == 'CAUTION':
                if not avoidance_active:
                    print(f"⚠️ AVOIDANCE: SLOWING. Obstacle @ {closest:.2f}m")
                    # Don't halt, but slow down — reduce RTL speed
                    # Send to AI for planning
                    if self.ws:
                        try:
                            await self.ws.send(json.dumps({
                                'type': 'avoidance_warning',
                                'safety': safety,
                                'mode': mode,
                            }))
                        except: pass

            elif level in ('WARN', 'CLEAR'):
                if avoidance_active:
                    print(f"✅ AVOIDANCE: CLEAR. Resuming {mode.upper()}")
                    avoidance_active = False
                    # Resume original mode
                    if mode == 'rth':
                        self.fc.set_mode('RTL')
                    elif mode == 'land':
                        self.fc.mav.command_long_send(
                            self.fc.target_system, self.fc.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)

            # Handle AI avoidance response (if received via command loop)
            ai_avoidance = self.telemetry_cache.get('ai_avoidance_vector', None)
            if ai_avoidance and avoidance_active:
                # AI gives velocity vector to avoid obstacle
                vx = ai_avoidance.get('vx', 0)
                vy = ai_avoidance.get('vy', 0)
                vz = ai_avoidance.get('vz', 0)

                # BUT ONLY if sensors still say it's safe in that direction
                tof_front = self.telemetry_cache.get('t1', 9999)
                tof_right = self.telemetry_cache.get('t2', 9999)
                tof_back = self.telemetry_cache.get('t3', 9999)
                tof_left = self.telemetry_cache.get('t4', 9999)

                # SENSOR PRIORITY: Block AI vector if sensor says obstacle in that direction
                if vx > 0 and tof_front < 500:  # Moving forward but front blocked
                    vx = 0
                    print("🛑 SENSOR OVERRIDE: AI wants forward, front ToF blocked")
                if vx < 0 and tof_back < 500:   # Moving backward but back blocked
                    vx = 0
                    print("🛑 SENSOR OVERRIDE: AI wants backward, back ToF blocked")
                if vy > 0 and tof_right < 500:  # Moving right but right blocked
                    vy = 0
                    print("🛑 SENSOR OVERRIDE: AI wants right, right ToF blocked")
                if vy < 0 and tof_left < 500:   # Moving left but left blocked
                    vy = 0
                    print("🛑 SENSOR OVERRIDE: AI wants left, left ToF blocked")

                if vx != 0 or vy != 0 or vz != 0:
                    # Apply AI avoidance vector via GUIDED mode velocity
                    self.fc.mav.set_mode_send(
                        self.fc.target_system,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4)  # GUIDED
                    self.fc.mav.set_position_target_local_ned_send(
                        0, self.fc.target_system, self.fc.target_component,
                        mavutil.mavlink.MAV_FRAME_BODY_NED,
                        0b0000111111000111,  # Velocity only
                        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)
                    print(f"🤖 AI AVOIDANCE: vx={vx:.1f} vy={vy:.1f} vz={vz:.1f} (sensor-validated)")

                # Clear the vector after applying
                self.telemetry_cache['ai_avoidance_vector'] = None

            await asyncio.sleep(0.1)  # 10Hz check rate

        print(f"🛡️ SMART AVOIDANCE MONITOR STOPPED (mode={mode})")

    async def video_loop(self):
        import cv2
        import subprocess

        print("🔍 V110: DUAL CAMERA INIT (Pi Camera V2 + GoPro Hero 12)...")

        # === INTERNAL CAMERA: Pi Camera V2 via CSI ===
        internal_pipeline = (
            "v4l2src device=/dev/video0 ! "
            "videoscale ! video/x-raw,width=1280,height=720 ! "
            "videoconvert ! video/x-raw,format=BGR ! "
            "appsink drop=true max-buffers=1 sync=false"
        )
        self.internal_cap = cv2.VideoCapture(internal_pipeline, cv2.CAP_GSTREAMER)
        if self.internal_cap and self.internal_cap.isOpened():
            ret, f = self.internal_cap.read()
            if ret and f is not None:
                print(f"✅ INTERNAL (Pi Camera V2): ACTIVE @ {f.shape[1]}x{f.shape[0]}")
            else:
                print("⚠️ INTERNAL: Opened but no frames yet")
        else:
            print("❌ INTERNAL: Failed to open /dev/video0. Will retry in loop.")
            self.internal_cap = None

        # === EXTERNAL CAMERA: GoPro Hero 12 via WiFi UDP ===
        # GoPro streams via WiFi to UDP port 8554 (set up via BLE command)
        await self._try_connect_gopro()

        self.cam_needs_reset = False
        sources = []
        if self.internal_cap: sources.append("Pi Camera V2")
        if self.external_cap: sources.append("GoPro Hero 12")
        self.telemetry_cache['cam_source'] = '+'.join(sources) if sources else 'none'
        self.telemetry_cache['active_cam'] = self.active_cam
        self.telemetry_cache['external_available'] = self.external_available
        print(f"📹 CAMERAS READY: {sources or 'NONE - will retry'}")

        if not self.internal_cap and not self.external_cap:
            print("🚨 NO CAMERAS DETECTED. Will keep retrying...")

        # Periodic GoPro reconnection attempt (backs off to reduce log spam)
        gopro_retry_interval = 30  # Start at 30s, back off to 120s
        last_gopro_retry = 0
        gopro_retry_count = 0
        # V110: DUAL CAMERA STREAM LOOP
        self.frame_in_transit = False
        self.udp_streamers_init = False
        self.ts_laptop_streamer = None
        self.ts_phone_streamer = None

        async with aiohttp.ClientSession() as session:
            while self.running:
                loop = asyncio.get_running_loop()
                current_time = int(time.time())

                # --- Periodic GoPro reconnect if not available (with backoff) ---
                if not self.external_available and current_time - last_gopro_retry > gopro_retry_interval:
                    last_gopro_retry = current_time
                    gopro_retry_count += 1
                    await self._try_connect_gopro()
                    # Back off: 30s → 60s → 120s max
                    gopro_retry_interval = min(120, 30 * gopro_retry_count)

                # --- Periodic internal cam reconnect if lost ---
                if self.internal_cap is None or not self.internal_cap.isOpened():
                    try:
                        internal_pipeline = (
                            "v4l2src device=/dev/video0 ! videoscale ! "
                            "video/x-raw,width=1280,height=720 ! videoconvert ! "
                            "video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false"
                        )
                        self.internal_cap = cv2.VideoCapture(internal_pipeline, cv2.CAP_GSTREAMER)
                    except: pass

                # --- Read from BOTH cameras (non-blocking) ---
                internal_frame = None
                external_frame = None

                if self.internal_cap and self.internal_cap.isOpened():
                    try:
                        ret, f = await loop.run_in_executor(None, self.internal_cap.read)
                        if ret and f is not None:
                            internal_frame = f
                            self.latest_internal_frame = f
                    except: pass

                if self.external_cap and self.external_cap.isOpened():
                    try:
                        ret, f = await loop.run_in_executor(None, self.external_cap.read)
                        if ret and f is not None:
                            external_frame = f
                            self.latest_external_frame = f
                        else:
                            # GoPro disconnected mid-flight
                            self.external_available = False
                            self.external_cap = None
                            self.telemetry_cache['external_available'] = False
                    except:
                        self.external_available = False

                # --- Select active frame for app preview ---
                if self.active_cam == 'external' and external_frame is not None:
                    active_frame = external_frame
                elif internal_frame is not None:
                    active_frame = internal_frame
                elif external_frame is not None:
                    active_frame = external_frame  # Fallback
                else:
                    await asyncio.sleep(0.1)
                    continue  # No frames from either camera

                # --- Select recording frame (may differ from active) ---
                if self.recording_cam == 'external' and external_frame is not None:
                    recording_frame = external_frame
                elif self.recording_cam == 'internal' and internal_frame is not None:
                    recording_frame = internal_frame
                else:
                    recording_frame = active_frame  # Fallback

                # Resize active frame to standard size
                if active_frame.shape[1] != CAM_WIDTH or active_frame.shape[0] != CAM_HEIGHT:
                    try:
                        active_frame = cv2.resize(active_frame, (CAM_WIDTH, CAM_HEIGHT), interpolation=cv2.INTER_LINEAR)
                    except: pass

                try:
                    # V87: Brightness boost for dark conditions
                    active_frame = cv2.convertScaleAbs(active_frame, alpha=1.5, beta=30)

                    # --- TAILSCALE UDP STREAMS (init once) ---
                    if not self.udp_streamers_init:
                        self.udp_streamers_init = True
                        if TAILSCALE_LAPTOP_IP:
                            try:
                                lp = ("appsrc ! videoconvert ! "
                                    "x264enc tune=zerolatency bitrate=2000 speed-preset=ultrafast key-int-max=30 ! "
                                    "mpegtsmux ! udpsink host=" + TAILSCALE_LAPTOP_IP + " port=" + str(TAILSCALE_VIDEO_PORT) + " sync=false")
                                self.ts_laptop_streamer = cv2.VideoWriter(lp, cv2.CAP_GSTREAMER, 0, 30.0, (1280, 720), True)
                                if not (self.ts_laptop_streamer and self.ts_laptop_streamer.isOpened()):
                                    lp = ("appsrc ! videoconvert ! avenc_mpeg4 bitrate=1500000 gop-size=15 ! "
                                        "avimux ! udpsink host=" + TAILSCALE_LAPTOP_IP + " port=" + str(TAILSCALE_VIDEO_PORT) + " sync=false")
                                    self.ts_laptop_streamer = cv2.VideoWriter(lp, cv2.CAP_GSTREAMER, 0, 30.0, (1280, 720), True)
                                print(f"📡 TAILSCALE → LAPTOP AI: {TAILSCALE_LAPTOP_IP}:{TAILSCALE_VIDEO_PORT}")
                            except Exception as e:
                                print(f"❌ Tailscale Laptop Stream: {e}")
                        if TAILSCALE_PHONE_IP:
                            try:
                                pp = ("appsrc ! videoconvert ! avenc_mpeg4 bitrate=800000 gop-size=15 ! "
                                    "avimux ! udpsink host=" + TAILSCALE_PHONE_IP + " port=" + str(TAILSCALE_VIDEO_PORT + 1) + " sync=false")
                                self.ts_phone_streamer = cv2.VideoWriter(pp, cv2.CAP_GSTREAMER, 0, 30.0, (640, 480), True)
                                print(f"📡 TAILSCALE → PHONE: {TAILSCALE_PHONE_IP}:{TAILSCALE_VIDEO_PORT + 1}")
                            except: pass

                    # Send RECORDING frame (not active) to AI via Tailscale
                    ai_frame = cv2.resize(recording_frame, (1280, 720)) if recording_frame.shape[:2] != (720, 1280) else recording_frame
                    if self.ts_laptop_streamer:
                        self.ts_laptop_streamer.write(ai_frame)
                    if self.ts_phone_streamer:
                        self.ts_phone_streamer.write(cv2.resize(recording_frame, (640, 480)))

                    # --- RECORDING (uses recording_cam frame, user-selected res) ---
                    if self.is_recording:
                        if self.video_writer is None:
                            fname = f"flight_record_{current_time}.mp4"
                            h, w = recording_frame.shape[:2]
                            print(f"📼 RECORDING: {fname} @ {w}x{h} cam={self.recording_cam}")
                            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                            self.video_writer = cv2.VideoWriter(fname, fourcc, 30.0, (w, h))
                            self.record_start_time = current_time
                        if self.video_writer:
                            self.video_writer.write(recording_frame)
                    else:
                        if self.video_writer is not None:
                            print(f"💾 SAVED ({int(current_time - self.record_start_time)}s)")
                            self.video_writer.release()
                            self.video_writer = None

                    # --- PHOTO CAPTURE (uses recording_cam frame) ---
                    if self.take_photo_flag:
                        fname = f"photo_{current_time}.jpg"
                        cv2.imwrite(fname, recording_frame)
                        print(f"📸 PHOTO: {fname}")
                        self.take_photo_flag = False

                    # --- APP PREVIEW (active_cam, downscaled, via Render relay) ---
                    if not self.frame_in_transit:
                        preview = cv2.resize(active_frame, (480, 360))
                        retval, buffer = cv2.imencode('.jpg', preview, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
                        if retval:
                            self.latest_jpeg = buffer.tobytes()
                            asyncio.create_task(self.push_frame(session, self.latest_jpeg))
                            if current_time % 5 == 0 and current_time != getattr(self, 'last_vid_sec', 0):
                                self.last_vid_sec = current_time
                                print(f"🎥 STREAM: active={self.active_cam} rec={self.recording_cam} ext={'✅' if self.external_available else '❌'}")

                except Exception as e:
                    print(f"⚠️ Stream Error: {e}")

                await asyncio.sleep(0.01)
    async def push_frame(self, session, data):
        if self.frame_in_transit: return
        self.frame_in_transit = True
        try:
            url = API_URL + "/video/frame"
            # Proper Multipart Upload
            form = aiohttp.FormData()
            form.add_field('file', data, filename='frame.jpg', content_type='image/jpeg')

            # V47: Increased Timeout for Weak Signals
            # V108: Increased to 10s for Larger Frames (Quality 60)
            async with session.post(url, data=form, timeout=10.0) as response:
                if response.status != 200:
                    print(f"❌ Video Upload Failed: {response.status}")
        except Exception as e:
            print(f"❌ Video Network Error: {repr(e)}")
        finally:
            self.frame_in_transit = False
    async def lidar_loop(self):
        """V103: YDLIDAR with SDK -> raw serial fallback"""
        import math

        # --- Try SDK first, fall back to raw serial parser ---
        use_sdk = False
        try:
            import ydlidar
            use_sdk = True
            print("✅ LIDAR: ydlidar SDK available")
        except ImportError:
            print("⚠️ LIDAR: SDK missing, trying raw serial fallback (lidar_driver.py)")

        if use_sdk:
            await self._lidar_loop_sdk(ydlidar, math)
        else:
            await self._lidar_loop_serial(math)

    def _send_lidar_to_fc(self, points_deg, math):
        """Send the FULL 360° LiDAR ring to the FC as OBSTACLE_DISTANCE (72 × 5° sectors) so ArduPilot's
        Proximity/AVOID knows the *direction* of every obstacle — NOT just the single nearest point
        mislabelled 'forward' (the old bug: a wall on the LEFT was reported as 40cm in FRONT, so the FC
        braked the wrong axis). points_deg = iterable of (angle_deg, dist_m) in the LiDAR's own frame.

        ⚙️ FC PARAM REQUIRED: PRX1_TYPE=2 (MAVLink proximity) to consume OBSTACLE_DISTANCE. (With the old
        PRX1_TYPE=4 the proximity read the single forward rangefinder — that is why rangefinder1 went to 0
        when PRX was on: proximity 'claimed' the rangefinder. Type 2 reads the MAVLink ring instead, so
        rangefinder1 is free to show the true FORWARD distance again from the DISTANCE_SENSOR below.)

        Direction convention is bench-calibratable: LIDAR_YAW_OFFSET_DEG rotates the ring, LIDAR_DIR (+1/-1)
        flips CW/CCW. Verify on Mission Planner's Proximity radar: an obstacle straight ahead must appear
        at the TOP; if it's mirrored, flip LIDAR_DIR; if rotated, adjust the offset."""
        if not self.fc:
            return
        MINCM, MAXCM = 10, 800
        NOOBS = MAXCM + 1                              # MAVLink spec: max+1 = 'no measurement' (ignored)
        sectors = [NOOBS] * 72                         # 72 × 5° = full 360°
        off = float(getattr(self, 'LIDAR_YAW_OFFSET_DEG', 0.0))
        dirn = int(getattr(self, 'LIDAR_DIR', -1))     # LiDAR CCW → ArduPilot CW (clockwise from nose)
        fwd_min = None
        for ang_deg, dist_m in points_deg:
            if not (0.1 < dist_m < 8.0):
                continue
            cm = int(max(MINCM, min(MAXCM, dist_m * 100)))
            ap = (off + dirn * ang_deg) % 360.0        # body frame, clockwise from forward (nose = 0°)
            idx = int((ap + 2.5) // 5) % 72            # nearest 5° sector
            if cm < sectors[idx]:
                sectors[idx] = cm
            fa = (ap + 180.0) % 360.0 - 180.0          # signed angle -180..180 for the 'forward' test
            if abs(fa) <= 15.0 and (fwd_min is None or cm < fwd_min):
                fwd_min = cm
        try:
            self.fc.mav.obstacle_distance_send(
                int(time.time() * 1e6),                            # time_usec
                mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,         # sensor_type = 0
                sectors,                                           # 72 distances (cm)
                5,                                                 # increment (deg per sector)
                MINCM, MAXCM,                                      # min/max (cm)
                0.0,                                               # increment_f (0 → use `increment`)
                0.0,                                               # angle_offset
                getattr(mavutil.mavlink, 'MAV_FRAME_BODY_FRD', 12) # body FRD: CW from forward
            )
        except Exception as e:
            if int(time.time()) % 10 == 0:
                print(f"⚠️ obstacle_distance_send failed: {e}")
        # TRUE forward distance (not the global min) → keeps rangefinder1 meaningful when RNGFND1_ORIENT=0.
        # time_boot_ms = real ms (Jul-2 diagnostic improvement, kept — was 0).
        if fwd_min is not None and MINCM < fwd_min <= MAXCM:
            try:
                self.fc.mav.distance_sensor_send(int(time.time() * 1000) & 0xFFFFFFFF,
                                                 MINCM, MAXCM, int(fwd_min), 0, 0, 0, 0)
            except Exception as _se:
                print(f"❌ LIDAR SEND FAILED: {_se}")
        # Jul-2 diagnostic (kept): periodic send counter in the journal proves the ring is flowing.
        self._dbg_send_count = getattr(self, '_dbg_send_count', 0) + 1
        if self._dbg_send_count % 40 == 0:
            _known = sum(1 for s in sectors if s <= MAXCM)
            print(f"📡 LIDAR RING #{self._dbg_send_count}: {_known}/72 sectors, fwd={fwd_min}cm fc={self.fc is not None}")

    async def _lidar_loop_sdk(self, ydlidar, math):
        """YDLIDAR via official Python SDK"""
        ports = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0"]
        laser = None

        print(f"🔭 LIDAR SDK: Scanning ports {ports}...")

        for port in ports:
            temp_laser = ydlidar.CYdLidar()
            temp_laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
            temp_laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 115200)
            temp_laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
            temp_laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
            temp_laser.setlidaropt(ydlidar.LidarPropScanFrequency, 5.0)
            temp_laser.setlidaropt(ydlidar.LidarPropSampleRate, 3)
            temp_laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)
            temp_laser.setlidaropt(ydlidar.LidarPropMaxRange, 8.0)
            temp_laser.setlidaropt(ydlidar.LidarPropMinRange, 0.1)

            if temp_laser.initialize():
                print(f"✅ LIDAR SDK: FOUND @ {port}")
                laser = temp_laser
                break
            else:
                print(f"   Lidar SDK failed on {port}")

        if laser is None:
            print("❌ LIDAR SDK: Init Failed on ALL ports. Falling back to raw serial...")
            await self._lidar_loop_serial(math)
            return

        print("⚠️ BYPASSING turnOn() health check (too sensitive)")
        print("✅ LIDAR SDK: Starting scan loop...")
        scan = ydlidar.LaserScan()

        while self.running:
            try:
                 ret = laser.doProcessSimple(scan)
                 if ret:
                     min_dist = 10.0
                     pts_deg = []
                     for i in range(scan.points.size()):
                         point = scan.points[i]
                         if point.range > 0.1 and point.range < min_dist:
                             min_dist = point.range
                         if point.range > 0.1:
                             pts_deg.append((math.degrees(point.angle), point.range))  # SDK angle = radians

                     # Full 360° ring → FC proximity (direction-aware), + true forward → rangefinder1.
                     self._send_lidar_to_fc(pts_deg, math)

                     self.telemetry_cache['obstacle_dist'] = min_dist
                     self.telemetry_cache['lidar_dist'] = min_dist
                     self.telemetry_cache['lidar_status'] = "ACTIVE_SDK"

                     lidar_points = []
                     for i in range(scan.points.size()):
                         pt = scan.points[i]
                         if 0.1 < pt.range < 8.0:
                             x = pt.range * math.cos(pt.angle)
                             y = pt.range * math.sin(pt.angle)
                             lidar_points.append([round(x, 2), round(y, 2)])

                     if lidar_points:
                         step = max(1, len(lidar_points) // 400)  # 400 pts/scan: precision cloud (was 200)
                         sampled = lidar_points[::step]
                         scan_msg = json.dumps({"type": "lidar_scan", "payload": {
                             "points": sampled,
                             # The points above are RAW (range*cos, range*sin). This bridge and
                             # the laptop grid each apply their OWN mirror/rotation to them, from
                             # different variables in different processes. If those disagree,
                             # ArduPilot avoids one way while the AI believes the obstacle is on
                             # the opposite side. Publish ours so the consumer can check.
                             "conv": {"dir": int(getattr(self, "LIDAR_DIR", -1)),
                                      "yaw_off": float(os.getenv("LIDAR_YAW_OFFSET_DEG", "0"))}}})
                         if self.ws:
                             try: await self.ws.send(scan_msg)
                             except: pass
                         if self.local_clients:
                             for c in list(self.local_clients):
                                 try: await c.send(scan_msg)
                                 except: pass

                 await asyncio.sleep(0.05)
            except Exception as e:
                print(f"Lidar SDK Error: {e}")
                await asyncio.sleep(1)

    async def _lidar_loop_serial(self, math):
        """Fallback: raw serial protocol parser (no SDK needed)"""
        try:
            from lidar_driver import YDLidarDriver
        except ImportError:
            # lidar_driver.py not found — lidar fully unavailable
            print("❌ LIDAR: Both SDK and serial driver unavailable. LIDAR OFFLINE.")
            self.telemetry_cache['lidar_status'] = "OFFLINE"
            self.telemetry_cache['lidar_dist'] = 0.5  # Conservative: assume obstacle nearby
            return

        driver = YDLidarDriver(port='/dev/ttyUSB0', baudrate=115200)
        if not driver.start():
            print("❌ LIDAR SERIAL: Failed to find lidar on any USB port. LIDAR OFFLINE.")
            self.telemetry_cache['lidar_status'] = "OFFLINE"
            self.telemetry_cache['lidar_dist'] = 0.5
            return

        print("✅ LIDAR SERIAL: Running via raw protocol parser (fallback mode)")
        self.telemetry_cache['lidar_status'] = "ACTIVE_SERIAL"

        while self.running:
            try:
                scan = driver.get_scan()
                if not scan:
                    await asyncio.sleep(0.1)
                    continue

                # Find minimum distance
                min_dist = 10.0
                lidar_points = []
                pts_deg = []
                for angle, dist in scan:
                    if 0.1 < dist < 8.0:
                        if dist < min_dist:
                            min_dist = dist
                        x = dist * math.cos(math.radians(angle))
                        y = dist * math.sin(math.radians(angle))
                        lidar_points.append([round(x, 2), round(y, 2)])
                        pts_deg.append((angle, dist))              # serial angle already in degrees

                # Full 360° ring → FC proximity (direction-aware), + true forward → rangefinder1.
                self._send_lidar_to_fc(pts_deg, math)

                self.telemetry_cache['obstacle_dist'] = min_dist
                self.telemetry_cache['lidar_dist'] = min_dist
                self.telemetry_cache['lidar_status'] = "ACTIVE_SERIAL"

                # === VISO: no-GPS INDOOR position from 2D LiDAR SLAM -> FC via VISION_POSITION_ESTIMATE.
                #     Gives the EKF a position (EK3_SRC=ExternalNav) -> Loiter/PosHold/GUIDED + arms without
                #     GPS. Only runs when there is NO GPS 3D fix (outdoors the real GPS supersedes it). ===
                try:
                    if int(self.telemetry_cache.get('gps_fix', 0) or 0) < 3:
                        if not hasattr(self, '_pose_est'):
                            from lidar_pose import LidarPoseEstimator
                            self._pose_est = LidarPoseEstimator()
                            self._pose_t = time.time()
                        _now = time.time(); _dt = max(0.02, min(0.5, _now - self._pose_t)); self._pose_t = _now
                        _hdg = float(self.telemetry_cache.get('heading', 0) or 0)
                        # Dead-reckoning prior in BODY frame (forward, right) from the EKF's NED
                        # velocity, rotated by heading. Zero while the EKF has no solution, which
                        # is the same behaviour as before -- but once it does, the prior tracks.
                        _vn = float(self.telemetry_cache.get('vel_n', 0.0) or 0.0)
                        _ve = float(self.telemetry_cache.get('vel_e', 0.0) or 0.0)
                        _hr = math.radians(_hdg)
                        _odom = (_vn * math.cos(_hr) + _ve * math.sin(_hr),      # body forward
                                 -_vn * math.sin(_hr) + _ve * math.cos(_hr))     # body right
                        px, py, pyaw = self._pose_est.update(scan, _hdg, dt=_dt, odom_vel=_odom)

                        # SLAM DIVERGENCE WATCHDOG.
                        # The correlative matcher can pin the pose in degenerate geometry -- a
                        # featureless corridor, or a map built while stationary -- because the
                        # scan scores highest at the position that created the map. Publishing a
                        # FROZEN position is the dangerous outcome: the EKF believes the aircraft
                        # is still while it flies away, and position hold then drives it further
                        # off correcting an error that is not real. If odometry says we have
                        # travelled and the pose has not, stop publishing rather than lie.
                        _sp = math.hypot(_odom[0], _odom[1])
                        _md = getattr(self, '_slam_moved', 0.0) + math.hypot(px - getattr(self, '_slam_px', px),
                                                                             py - getattr(self, '_slam_py', py))
                        _od = getattr(self, '_slam_odo', 0.0) + _sp * _dt
                        self._slam_px, self._slam_py = px, py
                        if _od > 3.0:                       # evaluate over ~3 m of travel
                            _stuck = _md < _od * 0.25       # pose moved under a quarter of it
                            if _stuck and not getattr(self, '_slam_bad', False):
                                self._slam_bad = True
                                print(f"⚠️ SLAM DIVERGENCE: odometry {_od:.1f} m but pose moved "
                                      f"{_md:.1f} m - position estimate looks pinned. Suspending "
                                      f"VISION_POSITION_ESTIMATE (featureless area?).")
                            elif not _stuck and getattr(self, '_slam_bad', False):
                                self._slam_bad = False
                                print("✓ SLAM tracking recovered - resuming VISION_POSITION_ESTIMATE")
                            _md = _od = 0.0
                        self._slam_moved, self._slam_odo = _md, _od
                        _publish_pose = not getattr(self, '_slam_bad', False)
                        _alt = float(self.telemetry_cache.get('altitude_baro',
                                     self.telemetry_cache.get('altitude', 0)) or 0)
                        # ArduPilot NED earth frame: x=North, y=East, z=Down. Our SLAM frame @heading0:
                        # x=right(East), y=forward(North) -> North=py, East=px, Down=-alt.
                        # ⚠️ frame mapping + EKF fusion need on-drone verification (Mission Planner).
                        if _publish_pose:
                            self.fc.mav.vision_position_estimate_send(
                                int(_now * 1e6), py, px, -_alt, 0.0, 0.0, math.radians(pyaw))
                            self.telemetry_cache['slam_pose'] = [round(px, 2), round(py, 2), round(pyaw, 1)]
                        else:
                            # Suspended by the divergence watchdog: better for the EKF to have no
                            # external position than a stale one it will act on.
                            self.telemetry_cache['slam_pose'] = None
                except Exception:
                    pass

                # Broadcast to laptop AI
                if lidar_points:
                    step = max(1, len(lidar_points) // 400)  # 400 pts/scan: precision cloud (was 200)
                    sampled = lidar_points[::step]
                    scan_msg = json.dumps({"type": "lidar_scan", "payload": {
                             "points": sampled,
                             # The points above are RAW (range*cos, range*sin). This bridge and
                             # the laptop grid each apply their OWN mirror/rotation to them, from
                             # different variables in different processes. If those disagree,
                             # ArduPilot avoids one way while the AI believes the obstacle is on
                             # the opposite side. Publish ours so the consumer can check.
                             "conv": {"dir": int(getattr(self, "LIDAR_DIR", -1)),
                                      "yaw_off": float(os.getenv("LIDAR_YAW_OFFSET_DEG", "0"))}}})
                    if self.ws:
                        try: await self.ws.send(scan_msg)
                        except: pass
                    if self.local_clients:
                        for c in list(self.local_clients):
                            try: await c.send(scan_msg)
                            except: pass

                await asyncio.sleep(0.05)
            except Exception as e:
                print(f"Lidar Serial Error: {e}")
                await asyncio.sleep(1)

        driver.stop()

        laser.turnOff()
        laser.disconnecting()

    async def watchdog_loop(self):
        """Monitors Cloud Connection Health + Battery Failsafe"""
        print("🐕 Watchdog Active")
        while self.running:
            # V110: FAILSAFE - Return to USER'S LAST LOCATION on disconnect (not launch point)
            last_msg_delta = time.time() - self.last_cloud_msg

            # Only trigger RTH if armed AND airborne (altitude > 1m)
            airborne = self.telemetry_cache.get('altitude', 0) > 1.0
            if last_msg_delta > 30.0 and not self.watchdog_triggered and self.is_armed and airborne:
                 self.watchdog_triggered = True
                 # Priority: Return to user GPS if available, else RTL to launch
                 if self.user_gps:
                     lat, lng = self.user_gps
                     print(f"⚠️ LOST CONNECTION ({int(last_msg_delta)}s)! RETURNING TO USER @ ({lat:.6f}, {lng:.6f})")
                     try:
                         self.fc.mav.set_mode_send(self.fc.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4)  # GUIDED
                         self.fc.mav.mission_item_int_send(
                             self.fc.target_system, self.fc.target_component,
                             0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                             mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 0, 0, 0, 0,
                             int(lat * 1e7), int(lng * 1e7), 15  # Return at 15m altitude
                         )
                     except: pass
                 else:
                     print(f"⚠️ LOST CONNECTION ({int(last_msg_delta)}s)! NO USER GPS — TRIGGERING RTL!")
                     try:
                         self.fc.mav.set_mode_send(self.fc.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6)  # RTL
                     except: pass

            _fa = time.time() - getattr(self, '_fc_msg_t', 0.0)
            self.telemetry_cache['fc_link'] = ('LOST' if (getattr(self, '_fc_msg_t', 0.0)
                                                          and _fa > 3.0) else 'OK')

            # V110: BATTERY FAILSAFE — use configurable threshold (not hardcoded 15%)
            current_batt = self.telemetry_cache.get('battery', 100)
            if self.is_armed and current_batt < self.batt_threshold and current_batt > 0:
                 if not getattr(self, 'low_batt_triggered', False):
                      self.low_batt_triggered = True
                      self.follow_me_active = False  # Stop follow on low batt
                      # Use configured RTH destination
                      if self.batt_rth_destination == 'user' and self.user_gps:
                          lat, lng = self.user_gps
                          print(f"⚠️ LOW BATTERY ({current_batt}% < {self.batt_threshold}%)! RETURNING TO USER @ ({lat:.6f}, {lng:.6f})")
                          try:
                              self.fc.mav.set_mode_send(self.fc.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4)
                              self.fc.mav.mission_item_int_send(
                                  self.fc.target_system, self.fc.target_component,
                                  0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                  mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 0, 0, 0, 0,
                                  int(lat * 1e7), int(lng * 1e7), 15
                              )
                          except: pass
                      else:
                          print(f"⚠️ LOW BATTERY ({current_batt}% < {self.batt_threshold}%)! TRIGGERING RTL!")
                          try:
                              self.fc.mav.set_mode_send(self.fc.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6)
                          except: pass

            if last_msg_delta < 2.0:
                self.watchdog_triggered = False
            await asyncio.sleep(1)

    async def esp32_hardware_loop(self):
        """Complete ESP32 Bidirectional Link (WiFi UDP Edition)"""
        import socket
        
        UDP_IP = "0.0.0.0" # Listen on all interfaces
        UDP_PORT = 8888
        
        print(f"🔌 ESP32: Binding UDP Port {UDP_PORT}...")
        
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((UDP_IP, UDP_PORT))
        sock.setblocking(False) 
        
        print(f"✅ ESP32: UDP Listener Active on {UDP_PORT}")
        
        esp32_addr = None # Will learn from incoming packets

        while self.running:
            try:
                # === RECEIVE: ESP32 → Radxa (UDP) ===
                try:
                    data_raw, addr = sock.recvfrom(4096)
                    esp32_addr = addr # Store for sending back commands
                    
                    line = data_raw.decode('utf-8').strip()
                    
                    if line and line.startswith('{'):
                         try:
                             data = json.loads(line)
                             
                             # Parse VL53L1X sensor data (t1,t2,t3,t4 in mm)
                             # 2 Top (Angled), 2 Bottom (Angled). YDLidar (Horizontal).
                             if 't1' in data and 't2' in data and 't3' in data and 't4' in data:
                                 # Filter Self-Collisions (<15cm) - e.g. Propellers/Legs
                                 raw_distances = [data['t1'], data['t2'], data['t3'], data['t4']]
                                 valid_distances = [d for d in raw_distances if d > 150 and d < 8000]
                                 
                                 # SAFETY PRIORITY: Active Braking Override
                                 # If any object is within 50cm (excluding self <15cm), STOP immediately.
                                 min_dist = min(valid_distances) if valid_distances else 9999
                                 if min_dist < 500: # 50cm
                                     print(f"🛑 CRITICAL PROXIMITY ({min_dist}mm)! FORCE BRAKE!")
                                     try:
                                         # Send Brake/Hold Mode
                                         self.fc.mav.set_mode_send(
                                             self.fc.target_system,
                                             mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                             17 # BRAKE (ArduPilot) / HOLD (PX4)
                                         )
                                     except: pass
                                 
                                 # Send to FC via MAVLink (Mapping Top/Bottom)
                                 # T1, T2 = Top (Mapped to UP/FORWARD_UP?) -> Using UP (24) and CUSTOM
                                 # B1, B2 = Bottom (Mapped to DOWN (25))
                                 
                                 # We map loosely for logs, but YDLIDAR handles Horizontal Ring 
                                 self.telemetry_cache['prox_alert'] = len(valid_distances) > 0
                                      
                                 # Log occasionally
                                 if int(time.time()) % 2 == 0:
                                      print(f"📡 VL53 (Filtered): {valid_distances} mm")
                         
                             # P3.3: Inject ESP32 IMU data into telemetry cache
                             # ax/ay/az = accelerometer (m/s²), gx/gy/gz = gyroscope (rad/s)
                             if 'ax' in data:
                                 self.telemetry_cache['esp32_ax'] = data.get('ax', 0)
                                 self.telemetry_cache['esp32_ay'] = data.get('ay', 0)
                                 self.telemetry_cache['esp32_az'] = data.get('az', 0)
                             if 'gx' in data:
                                 self.telemetry_cache['esp32_gx'] = data.get('gx', 0)
                                 self.telemetry_cache['esp32_gy'] = data.get('gy', 0)
                                 self.telemetry_cache['esp32_gz'] = data.get('gz', 0)

                             # Inject raw ToF into telem cache for SafetyEnvelope
                             self.telemetry_cache['t1'] = data.get('t1', -1)
                             # Arrival stamp: the safety gate must not keep trusting these
                             # clearances after the ESP32 link drops.
                             self.telemetry_cache['_tof_t'] = time.time()
                             self.telemetry_cache['t2'] = data.get('t2', -1)
                             self.telemetry_cache['t3'] = data.get('t3', -1)
                             self.telemetry_cache['t4'] = data.get('t4', -1)

                             # P3.4: Vibration monitoring from ESP32 IMU
                             # If vibration is excessive, warn — may indicate prop damage
                             az = data.get('az', 9.8)
                             if abs(az) > 20:  # Normal gravity ~9.8, >20 = severe vibration
                                 print(f"⚠️ VIBRATION ALERT: az={az:.1f} m/s² (check props!)")
                                 self.telemetry_cache['vibration_alert'] = True
                             else:
                                 self.telemetry_cache['vibration_alert'] = False

                             # Forward full telemetry to cloud AND local clients (Tailscale direct)
                             esp32_msg = json.dumps({"type": "esp32_telem", "payload": data})
                             if self.ws:
                                 await self.ws.send(esp32_msg)
                             # Also send to local Tailscale clients (laptop AI direct path)
                             if self.local_clients:
                                 for c in list(self.local_clients):
                                     try: await c.send(esp32_msg)
                                     except: self.local_clients.discard(c)
                         
                         except Exception as e:
                             print(f"⚠️ ESP32 parse error: {e}")
                except BlockingIOError:
                    pass # No data waiting
                
                # === SEND: Radxa → ESP32 (UDP) ===
                if esp32_addr and hasattr(self, 'esp32_cmd_queue') and not self.esp32_cmd_queue.empty():
                    cmd = self.esp32_cmd_queue.get_nowait()
                    # MA-9 FIX: Convert gimbal commands to ESP32's expected format
                    # ESP32 firmware expects {"gim": [pitch, yaw]} not {"type":"gimbal","pitch":X,"yaw":Y}
                    if cmd.get('type') == 'gimbal':
                        esp32_msg = {"gim": [int(cmd.get('pitch', 0)), int(cmd.get('yaw', 0))]}
                    elif cmd.get('type') == 'led':
                        esp32_msg = {"led": cmd.get('color', 'OFF')}
                    elif cmd.get('type') == 'capture':
                        esp32_msg = {"capture": 1}
                    elif cmd.get('type') == 'record':
                        esp32_msg = {"record": 1 if cmd.get('recording') else 0}
                    else:
                        esp32_msg = cmd  # Pass through unknown commands as-is
                    msg = (json.dumps(esp32_msg) + '\n').encode('utf-8')
                    sock.sendto(msg, esp32_addr)
                    print(f"📤 ESP32 CMD: {esp32_msg} -> {esp32_addr}")
                
                await asyncio.sleep(0.01)
            except Exception as e:
                print(f"❌ ESP32 loop error: {e}")
                await asyncio.sleep(0.1)

if __name__ == "__main__":
    bridge = RadxaBridge()
    try:
        async def main():
            # Run everything in parallel so Video/Cloud doesn't wait for FC
            await asyncio.gather(
                bridge.connect_mavlink(),
                bridge.connect_cloud(),
                bridge.telemetry_loop(),  # Runs independently — NOT tied to cloud WS
                bridge.video_loop(),
                bridge.lidar_loop(),
                bridge.esp32_hardware_loop(),
                bridge.watchdog_loop(),
                bridge.start_local_server(),
                bridge.start_local_video_server(),
                # GoPro init used to live inside connect_cloud, so making the cloud relay
                # opt-in silently disabled the camera entirely. It belongs on the local path.
                bridge.init_gopro_proxy()
            )
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Stopping Bridge...")
