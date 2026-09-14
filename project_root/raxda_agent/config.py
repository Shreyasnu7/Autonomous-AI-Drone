# radxa_agent/config.py
# Configuration for Radxa Safety Gateway

SERVER_BASE = "https://drone-server-r0qe.onrender.com"  # Render server
PLAN_NEXT_ENDPOINT = f"{SERVER_BASE}/plan/next"
AUTH_TOKEN = "SUPER_SECRET_DRONE_KEY_123"   # must match server
WS_URL = SERVER_BASE.replace("https://", "wss://") + "/ws/connect/radxa"

# Persistent/local paths
PERSISTENT_JOB_DIR = "/mnt/data/jobs"  # server persistent path mirrored or used server-side
LAST_EXECUTED_PATH = "/tmp/last_executed_plan.json"

# Safety limits (tunable)
MAX_RADIUS_M = 50.0
MIN_RADIUS_M = 2.0
MAX_ALT_M = 120.0
MIN_ALT_M = 1.0
MAX_SPEED_MS = 8.0
MAX_YAW_DEG_S = 90.0

# Telemetry thresholds
MIN_BATTERY = 0.15    # 15%
MIN_SATS = 5

# Polling intervals
PLAN_POLL_INTERVAL = 2.0
TELEMETRY_PUBLISH_INTERVAL = 1.0

# Use MAVSDK telemetry if available on Radxa. Set to False to force simulation.
USE_MAVSDK = False
MAVSDK_CONNECTION_STR = "serial:///dev/ttyS0:921600"  # update if using MAVSDK
