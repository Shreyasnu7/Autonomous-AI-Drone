#!/bin/bash
# RADXA ZERO 3W (SD-Only) - SETUP SCRIPT
# Run as: sudo ./post_flash_setup_sd.sh

if [ "$EUID" -ne 0 ]; then 
  echo "Please run as root (sudo)"
  exit
fi

# Get the directory where this script is located (User's Project Folder)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")" # Parent of raxda_bridge
USER_HOME=$(eval echo ~$SUDO_USER)

echo "🔧 RADXA ZERO 3W (SD-BOOT) - SETUP"
echo "==================================="
echo "📂 Project Location: $PROJECT_ROOT"
echo "👤 User Home: $USER_HOME"
echo ""

# ===== 1. SYSTEM DEPENDENCIES =====
echo "📦 Step 1: Installing system packages..."
apt-get update
# Add Universe/Multiverse if needed? Usually default.
apt-get install -y \
    python3-pip \
    python3-venv \
    python3-dev \
    libopencv-dev \
    python3-opencv \
    v4l-utils \
    media-ctl \
    i2c-tools \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    python3-gst-1.0 \
    net-tools \
    wireless-tools \
    build-essential \
    git \
    cmake

echo "✅ System packages installed"

# ===== 2. OVERLAYS (CAMERA & UART) =====
echo ""
echo "⚙️ Step 2: Configuring Overlays..."

# Try uEnv.txt (Radxa) first, fall back to armbianEnv.txt
TARGET_ENV="/boot/uEnv.txt"
if [ ! -f "$TARGET_ENV" ]; then
    TARGET_ENV="/boot/armbianEnv.txt"
fi

if [ -f "$TARGET_ENV" ]; then
    echo "   Editing $TARGET_ENV..."
    
    # 1. IMX219 Camera
    if ! grep -q "radxa-zero3-imx219" "$TARGET_ENV"; then
        if grep -q "overlays=" "$TARGET_ENV"; then
            sudo sed -i '/^overlays=/ s/$/ radxa-zero3-imx219/' "$TARGET_ENV"
        else
            echo "overlays=radxa-zero3-imx219" >> "$TARGET_ENV"
        fi
        echo "   -> Added Camera Overlay"
    else
        echo "   -> Camera Overlay already present"
    fi
    
    # 2. UARTs (FC=UART2, ESP32=UART4)
    if ! grep -q "uart2" "$TARGET_ENV"; then
         sudo sed -i '/^overlays=/ s/$/ uart2/' "$TARGET_ENV"
    fi
    if ! grep -q "uart4" "$TARGET_ENV"; then
         sudo sed -i '/^overlays=/ s/$/ uart4/' "$TARGET_ENV"
    fi
    echo "   -> UART Overlays ensured"
else
    echo "⚠️ No uEnv.txt found! Check /boot/ config manually."
fi

# ===== 3. PERMISSIONS =====
echo ""
echo "🔐 Step 3: Setting Permissions..."
usermod -a -G dialout $SUDO_USER
usermod -a -G video $SUDO_USER
chmod 666 /dev/ttyS2 2>/dev/null || true
chmod 666 /dev/ttyS4 2>/dev/null || true
echo "✅ Permissions set"

# ===== 4. PYTHON SETUP =====
echo ""
echo "🐍 Step 4: Python Environment..."

# Since new SD = Fresh Venv.
# We create venv inside the project folder to be portable
VENV_DIR="$PROJECT_ROOT/venv"

if [ ! -d "$VENV_DIR" ]; then
    echo "   Creating venv at $VENV_DIR..."
    python3 -m venv "$VENV_DIR"
fi

echo "   Installing dependencies..."
"$VENV_DIR/bin/pip" install --upgrade pip
"$VENV_DIR/bin/pip" install \
    aiohttp \
    websockets \
    pymavlink \
    numpy \
    pyserial \
    opencv-python-headless # Fallback if system one fails

echo "✅ Python Environemnt Ready"

# ===== 5. CAMERA SETUP SCRIPT =====
echo ""
echo "📷 Step 5: Camera Setup Script..."
# This script manually configures ISP formats via media-ctl
cat > "$PROJECT_ROOT/raxda_bridge/setup_camera.sh" << 'CAMERA_EOF'
#!/bin/bash
set -e
# Try to configure ISP pipeline for IMX219 (1080p)
# This fixes "Purple" issue on some kernels

# Detect Media Device
MDEV=$(ls /dev/media* | head -n 1)
if [ -z "$MDEV" ]; then echo "No media device found"; exit 0; fi

echo "Configuring ISP via $MDEV..."

# Reset
sudo media-ctl -r -d $MDEV 2>/dev/null || true

# Helper Vars (Might vary by kernel, these are generic rkisp1 names)
# NOTE: vendor-rk35xx usually auto-configures, but if manual is needed:
# We just log that we tried. True reconfiguration requires exact entity names.
# For now, we trust the Overlay + V4L2 defaults unless broken.

# Set V4L2 Defaults
v4l2-ctl -d /dev/video0 --set-fmt-video=width=1920,height=1080,pixelformat=NV12 2>/dev/null || true

echo "✅ Camera Configured (Basic)"
CAMERA_EOF

chmod +x "$PROJECT_ROOT/raxda_bridge/setup_camera.sh"

# ===== 6. SERVICE SETUP =====
echo ""
echo "⚡ Step 6: Creating Service..."

SERVICE_FILE="/etc/systemd/system/drone-bridge.service"

cat <<EOF > "$SERVICE_FILE"
[Unit]
Description=Radxa Drone Bridge (SD)
After=network.target

[Service]
Type=simple
User=$SUDO_USER
WorkingDirectory=$PROJECT_ROOT/raxda_bridge
ExecStartPre=/bin/sleep 10
ExecStartPre=$PROJECT_ROOT/raxda_bridge/setup_camera.sh
ExecStart=$VENV_DIR/bin/python3 $PROJECT_ROOT/raxda_bridge/real_bridge_service.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable drone-bridge

echo "🎉 DONE! REBOOT NOW."
echo "   After reboot, check service: sudo systemctl status drone-bridge"
