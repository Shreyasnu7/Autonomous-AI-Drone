# laptop_ai/config.py
import os
import subprocess

# ===== CONNECTION MODE =====
# Priority: TAILSCALE (fastest) → RENDER (fallback)
# Tailscale IPs (from tailscale admin)
CUBIE_TAILSCALE_IP = os.getenv("CUBIE_TS_IP", "100.89.83.125")
LAPTOP_TAILSCALE_IP = os.getenv("LAPTOP_TS_IP", "100.84.75.22")

# Auto-detect: is Tailscale running and can we reach the Cubie?
# Use a real TCP connection (ICMP ping is often blocked over Tailscale/networks).
def _check_tailscale():
    import socket
    for port in (8000, 8080):  # bridge WS + video server
        try:
            s = socket.create_connection((CUBIE_TAILSCALE_IP, port), timeout=2)
            s.close()
            return True
        except Exception:
            continue
    return False

USE_TAILSCALE = _check_tailscale()

if USE_TAILSCALE:
    # TAILSCALE MODE — direct connection, <50ms latency
    VPS_WS = os.getenv("VPS_WS_URL", f"ws://{CUBIE_TAILSCALE_IP}:8000")
    API_BASE = os.getenv("API_BASE", f"http://{CUBIE_TAILSCALE_IP}:8080")
    print(f"🔗 CONNECTION: TAILSCALE DIRECT (ws://{CUBIE_TAILSCALE_IP}:8000)")
else:
    # RENDER MODE — cloud relay, 200-500ms latency
    VPS_WS = os.getenv("VPS_WS_URL", "wss://drone-server-r0qe.onrender.com/ws/connect/laptop_vision")
    API_BASE = os.getenv("API_BASE", "https://drone-server-r0qe.onrender.com")
    print(f"🌐 CONNECTION: RENDER CLOUD RELAY (slower, Tailscale not available)")

# Shared secret token (must match server)
AUTH_TOKEN = os.getenv("AUTH_TOKEN", "SUPER_SECRET_DRONE_KEY_123")

# OpenAI / GPT config
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY", "")
OPENAI_MODEL = os.getenv("OPENAI_MODEL", "gpt-4o-mini")

# DeepSeek / Local LLM Config (For Ultra-Low Latency Reasoning)
DEEPSEEK_API_KEY = os.getenv("DEEPSEEK_API_KEY", "sk-...")
DEEPSEEK_URL = os.getenv("DEEPSEEK_URL", "https://api.deepseek.com/v1")
USE_LOCAL_LLM = os.getenv("USE_LOCAL_LLM", "False").lower() == "true"

# YOLO model path (we default to yolov8n.pt). Put the .pt file in laptop_ai/ or change path.
YOLO_MODEL_PATH = os.getenv("YOLO_MODEL_PATH", "yolov8n.pt")

# RTSP / camera input for laptop
_rtsp = os.getenv("RTSP_URL", "0")
RTSP_URL = int(_rtsp) if _rtsp.isdigit() else _rtsp

# Camera Resolution Config (High Res Logic - Downscaled for Stream)
# 5.3K = 5312 x 2988 (Recording / AI Analysis)
CAM_WIDTH = 5312  
CAM_HEIGHT = 2988
CAM_FPS = 30

# Performance config
AI_CALL_INTERVAL = 2.0   # Fast re-planning for maximum responsiveness
FRAME_SKIP = 1           # PROCESS EVERY SINGLE FRAME (Cinema Quality)
TEMPORAL_SMOOTHING = 0.7 # Increased smoothing for steadycam feel

# artifact dir
TEMP_ARTIFACT_DIR = "./artifacts"
MAX_IMAGES_UPLOAD = 25
MAX_VIDEO_UPLOAD_MB = 100
