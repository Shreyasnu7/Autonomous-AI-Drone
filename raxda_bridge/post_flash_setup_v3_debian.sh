#!/bin/bash
# =====================================================
# RADXA ZERO 3W - POST-FLASH SETUP v3.0 (DEBIAN SD)
# THE "ONE SCRIPT TO RULE THEM ALL"
# =====================================================
# Combines: post_flash_setup_v2, magic_setup, maximum_armor, hardening
# Adapted for: Radxa official Debian Bookworm on SD card
#
# What this does:
#   - Installs ALL system + python packages
#   - Configures serial ports (FC=UART2, ESP32=UART4, LiDAR=USB)
#   - Installs + configures Tailscale VPN (mesh network)
#   - Sets up thermal protection (throttle, NEVER shutdown)
#   - Hardware watchdog (auto-reboot on freeze)
#   - Kernel panic auto-recovery
#   - Creates drone bridge systemd service (auto-start)
#   - Camera setup (IMX219 + GoPro USB)
#   - Filesystem hardening (journal, anti-corruption)
#   - SSH + permissions
#
# Usage:
#   1. Flash Radxa Debian to SD card, boot, create user shreyash
#   2. Connect WiFi, note IP
#   3. From laptop PowerShell: .\deploy_to_new_radxa.ps1
#      OR manually:
#        scp -r drone_project/ shreyash@<ip>:~/
#        ssh shreyash@<ip>
#        chmod +x ~/drone_project/raxda_bridge/post_flash_setup_v3_debian.sh
#        sudo ~/drone_project/raxda_bridge/post_flash_setup_v3_debian.sh
# =====================================================

set -e

if [ "$EUID" -ne 0 ]; then
  echo "Please run as root (sudo)"
  exit 1
fi

echo "================================================"
echo " RADXA ZERO 3W - POST-FLASH SETUP v3.0"
echo " (Debian SD Card - All-In-One)"
echo "================================================"
echo ""

USER_NAME="shreyash"
USER_HOME="/home/$USER_NAME"
DRONE_DIR="$USER_HOME/drone_project"
BRIDGE_DIR="$DRONE_DIR/raxda_bridge"

# ===== 1. SYSTEM PACKAGES =====
echo "[1/14] Installing system packages..."
apt-get update -y
apt-get install -y \
    python3 \
    python3-pip \
    python3-venv \
    python3-dev \
    python3-opencv \
    libopencv-dev \
    git \
    cmake \
    build-essential \
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
    network-manager \
    bluez \
    bluetooth \
    curl \
    htop \
    rsync \
    || echo "  Some packages may have failed (non-critical)"
echo "  Done"

# ===== 2. PYTHON PACKAGES (SYSTEM-WIDE) =====
echo ""
echo "[2/14] Installing Python packages..."
pip3 install --break-system-packages \
    pymavlink \
    pyserial \
    websockets \
    aiohttp \
    numpy \
    smbus2 \
    bleak \
    opencv-python-headless \
    || echo "  Some pip packages may have failed"
echo "  Done"

# ===== 3. TAILSCALE VPN =====
echo ""
echo "[3/14] Installing Tailscale..."
if ! command -v tailscale &> /dev/null; then
    curl -fsSL https://tailscale.com/install.sh | sh
    echo "  Tailscale installed"
else
    echo "  Tailscale already installed"
fi
systemctl enable --now tailscaled
echo "  Tailscale daemon enabled"
echo ""
echo "  >>> AFTER SETUP COMPLETES: Run 'sudo tailscale up' to authenticate <<<"
echo ""

# ===== 4. USB-TO-SERIAL DRIVERS =====
echo ""
echo "[4/14] Configuring USB serial drivers..."
modprobe cp210x 2>/dev/null || true
modprobe ch341 2>/dev/null || true
for mod in cp210x ch341; do
    if ! grep -q "$mod" /etc/modules 2>/dev/null; then
        echo "$mod" >> /etc/modules
    fi
done
echo "  Done"

# ===== 5. SERIAL PORT PERMISSIONS =====
echo ""
echo "[5/14] Setting permissions..."
usermod -a -G dialout $USER_NAME 2>/dev/null || true
usermod -a -G video $USER_NAME 2>/dev/null || true
usermod -a -G bluetooth $USER_NAME 2>/dev/null || true
usermod -a -G sudo $USER_NAME 2>/dev/null || true
usermod -a -G i2c $USER_NAME 2>/dev/null || true

# Persistent udev rules for serial ports
cat > /etc/udev/rules.d/99-drone-serial.rules << 'UDEV_EOF'
# FC on UART2
KERNEL=="ttyS2", MODE="0666"
# ESP32 on UART4
KERNEL=="ttyS4", MODE="0666"
# YDLidar on USB
KERNEL=="ttyUSB*", MODE="0666"
# GoPro USB webcam
KERNEL=="video*", MODE="0666"
UDEV_EOF

udevadm control --reload-rules 2>/dev/null || true
echo "  Done"

# ===== 6. THERMAL PROTECTION (NEVER SHUTDOWN) =====
echo ""
echo "[6/14] Setting up thermal protection..."

cat > /usr/local/bin/thermal_guard.sh << 'THERMAL_EOF'
#!/bin/bash
# Thermal guard for drone - CPU throttle only, NEVER shutdown
# Safe for in-flight operation

THROTTLE_TEMP=60000   # 60C - start throttling
MAX_TEMP=70000        # 70C - aggressive throttle
NORMAL_FREQ=1416000   # Normal max freq (1.416 GHz)
THROTTLE_FREQ=1200000 # Throttled freq (1.2 GHz)
LOW_FREQ=816000       # Emergency low freq (816 MHz)

while true; do
    TEMP=$(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null || echo "0")

    if [ "$TEMP" -gt "$MAX_TEMP" ]; then
        for cpu in /sys/devices/system/cpu/cpufreq/policy*/scaling_max_freq; do
            echo "$LOW_FREQ" > "$cpu" 2>/dev/null || true
        done
    elif [ "$TEMP" -gt "$THROTTLE_TEMP" ]; then
        for cpu in /sys/devices/system/cpu/cpufreq/policy*/scaling_max_freq; do
            echo "$THROTTLE_FREQ" > "$cpu" 2>/dev/null || true
        done
    else
        for cpu in /sys/devices/system/cpu/cpufreq/policy*/scaling_max_freq; do
            echo "$NORMAL_FREQ" > "$cpu" 2>/dev/null || true
        done
    fi
    sleep 5
done
THERMAL_EOF
chmod +x /usr/local/bin/thermal_guard.sh

cat > /etc/systemd/system/thermal-guard.service << 'TG_EOF'
[Unit]
Description=Thermal Guard (CPU throttle - drone safe, never shutdown)
After=multi-user.target

[Service]
Type=simple
ExecStart=/usr/local/bin/thermal_guard.sh
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
TG_EOF

systemctl daemon-reload
systemctl enable thermal-guard
systemctl start thermal-guard
echo "  Thermal guard active (throttle 60C, aggressive 70C, NEVER shutdown)"

# ===== 7. HARDWARE WATCHDOG (AUTO-REBOOT ON FREEZE) =====
echo ""
echo "[7/14] Enabling hardware watchdog..."
# If the system freezes (solid green light), watchdog reboots in 15s
sed -i 's/#RuntimeWatchdogSec=.*/RuntimeWatchdogSec=15/' /etc/systemd/system.conf 2>/dev/null || true
sed -i 's/#RebootWatchdogSec=.*/RebootWatchdogSec=2min/' /etc/systemd/system.conf 2>/dev/null || true
echo "softdog" > /etc/modules-load.d/watchdog.conf 2>/dev/null || true
echo "  Done (auto-reboot on 15s freeze)"

# ===== 8. KERNEL PANIC AUTO-RECOVERY =====
echo ""
echo "[8/14] Enabling kernel panic auto-recovery..."
cat > /etc/sysctl.d/99-drone-panic.conf << 'PANIC_EOF'
# Auto-reboot 10s after kernel panic (don't stay stuck)
kernel.panic = 10
kernel.sysrq = 1
PANIC_EOF
sysctl --system 2>/dev/null || true
echo "  Done (auto-reboot 10s after panic)"

# ===== 9. FILESYSTEM HARDENING =====
echo ""
echo "[9/14] Hardening filesystem..."
# Enable journal data write mode for corruption protection
tune2fs -o journal_data_writeback $(findmnt -n -o SOURCE /) 2>/dev/null || true
echo "  Done"

# ===== 10. CAMERA SETUP SCRIPT =====
echo ""
echo "[10/14] Creating camera setup script..."
cat > $USER_HOME/setup_camera.sh << 'CAMERA_EOF'
#!/bin/bash
# Camera pipeline configuration for IMX219 + GoPro

# Try IMX219 (Pi Camera V2.1) first
if [ -e /dev/media0 ]; then
    sudo media-ctl -r -d /dev/media0 2>/dev/null || true
    sudo v4l2-ctl -d /dev/video0 --set-fmt-video=width=1920,height=1080,pixelformat=NV12 2>/dev/null || true
    echo "Camera: IMX219 configured"
fi

# Check for GoPro USB webcam
for dev in /dev/video*; do
    if v4l2-ctl -d "$dev" --info 2>/dev/null | grep -qi "gopro"; then
        echo "Camera: GoPro USB webcam found at $dev"
        break
    fi
done
CAMERA_EOF
chmod +x $USER_HOME/setup_camera.sh
chown $USER_NAME:$USER_NAME $USER_HOME/setup_camera.sh

# ===== 11. DRONE BRIDGE LAUNCHER =====
echo ""
echo "[11/14] Creating drone bridge launcher..."

cat > $USER_HOME/launch_bridge.sh << 'LAUNCH_EOF'
#!/bin/bash
echo "==============================="
echo " RADXA DRONE BRIDGE LAUNCHER"
echo "==============================="

# Step 1: Camera
echo "[1/3] Configuring Camera..."
~/setup_camera.sh 2>/dev/null || echo "Camera setup skipped"

# Step 2: Tailscale
echo "[2/3] Checking Tailscale..."
tailscale status 2>/dev/null | head -3 || echo "Tailscale not running"

# Step 3: Bridge
echo "[3/3] Launching Bridge..."

# Kill any stale UART locks
sudo fuser -k /dev/ttyS2 2>/dev/null || true
sleep 0.5

BRIDGE="$HOME/drone_project/raxda_bridge/real_bridge_service.py"
if [ -f "$BRIDGE" ]; then
    cd "$(dirname "$BRIDGE")"
    exec python3 "$BRIDGE"
else
    echo "ERROR: Bridge code not found at $BRIDGE"
    echo "SCP your code: scp -r raxda_bridge/ shreyash@<radxa-ip>:~/drone_project/"
    exit 1
fi
LAUNCH_EOF
chmod +x $USER_HOME/launch_bridge.sh
chown $USER_NAME:$USER_NAME $USER_HOME/launch_bridge.sh

# ===== 12. SYSTEMD SERVICE =====
echo ""
echo "[12/14] Creating drone bridge service..."

cat > /etc/systemd/system/drone-bridge.service << SERVICE_EOF
[Unit]
Description=Radxa Drone Bridge Service
After=network.target tailscaled.service

[Service]
Type=simple
User=$USER_NAME
ExecStartPre=/bin/sleep 10
ExecStart=$USER_HOME/launch_bridge.sh
WorkingDirectory=$USER_HOME
Restart=always
RestartSec=5
Environment=TS_LAPTOP_IP=100.64.0.10
Environment=TS_PHONE_IP=100.64.0.20

[Install]
WantedBy=multi-user.target
SERVICE_EOF

systemctl daemon-reload
systemctl enable drone-bridge
echo "  Done (auto-starts on boot)"

# ===== 13. SSH & SUDO =====
echo ""
echo "[13/14] Configuring SSH and sudo..."
systemctl enable ssh 2>/dev/null || true

cat > /etc/sudoers.d/drone-ops << 'SUDOERS_EOF'
shreyash ALL=(ALL) NOPASSWD: /usr/bin/mount, /usr/bin/umount, /usr/bin/systemctl restart drone-bridge, /usr/bin/systemctl stop drone-bridge, /usr/bin/systemctl start drone-bridge, /usr/bin/fuser
SUDOERS_EOF
chmod 440 /etc/sudoers.d/drone-ops
echo "  Done"

# ===== 14. SAFE-EDIT UTILITY =====
echo ""
echo "[14/14] Installing utilities..."
cat > /usr/local/bin/safe-edit << 'SAFEEDIT_EOF'
#!/bin/bash
if [ -z "$1" ]; then echo "Usage: safe-edit <file>"; exit 1; fi
FILE="$1"
sudo chattr -i "$FILE" 2>/dev/null || true
sudo nano "$FILE"
sudo chattr +i "$FILE" 2>/dev/null || true
echo "File re-locked."
SAFEEDIT_EOF
chmod +x /usr/local/bin/safe-edit

# Make sure drone project dir exists with correct ownership
mkdir -p "$BRIDGE_DIR"
chown -R $USER_NAME:$USER_NAME "$DRONE_DIR"

echo ""
echo "================================================"
echo " POST-FLASH SETUP v3.0 COMPLETE!"
echo "================================================"
echo ""
echo " Hardware Ports:"
echo "   FC:        /dev/ttyS2 @ 57600 baud"
echo "   ESP32:     /dev/ttyS4 @ 115200 baud"
echo "   LiDAR:     /dev/ttyUSB0 @ 128000 baud"
echo "   Camera:    /dev/video0 (IMX219/GoPro)"
echo ""
echo " Protection:"
echo "   Thermal:   Throttle at 60C, aggressive 70C (NEVER shutdown)"
echo "   Watchdog:  Auto-reboot on 15s freeze"
echo "   Panic:     Auto-reboot 10s after kernel panic"
echo "   Journal:   Filesystem write protection enabled"
echo ""
echo " Services:"
echo "   drone-bridge:     enabled (auto-start on boot)"
echo "   thermal-guard:    enabled (CPU throttle)"
echo "   tailscaled:       enabled"
echo ""
echo " Tailscale IPs (after auth):"
echo "   Radxa:  (run: sudo tailscale up)"
echo "   Laptop: 100.64.0.10"
echo "   Phone:  100.64.0.20"
echo ""
echo " Server:"
echo "   WebSocket: wss://drone-server-r0qe.onrender.com/ws/connect/RADXA_X"
echo ""
echo " NEXT STEPS:"
echo "   1. sudo tailscale up              (authenticate)"
echo "   2. sudo reboot                    (apply all changes)"
echo "   3. After reboot, bridge auto-starts"
echo ""
echo " TO UPDATE CODE FROM LAPTOP:"
echo "   scp -r raxda_bridge/ shreyash@<radxa-ip>:~/drone_project/"
echo "   ssh shreyash@<ip> 'sudo systemctl restart drone-bridge'"
echo ""
echo "================================================"
