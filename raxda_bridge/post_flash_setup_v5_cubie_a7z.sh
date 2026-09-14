#!/bin/bash
# =====================================================
# RADXA CUBIE A7Z - POST-FLASH SETUP v5.0
# For: Radxa Debian 11 CLI on SD card
# =====================================================
# Allwinner A733 | WiFi 6 (AIC8800) | BT 5.4
# Direct drop-in replacement for Radxa Zero 3W
#
# PCB Connection:
#   Pin 2  (5V)  → Drone 5V BEC
#   Pin 8  (TX)  → FC UART RX (UART0 on A733)
#   Pin 10 (RX)  → FC UART TX
#   Pin 25 (GND) → Drone GND
#   USB-C 2.0    → USB Hub (YDLIDAR + 4G dongle)
#   CSI          → Pi Camera V2 (IMX219)
#   WiFi         → ESP32 (sensor data UDP)
#   BLE          → GoPro Hero 12
#
# Usage:
#   1. Flash Radxa Debian 11 CLI to SD card
#   2. Boot, create user shreyash (pw: 1)
#   3. Connect WiFi: nmcli dev wifi connect "SSID" password "PASS"
#   4. SCP this script: scp post_flash_setup_v5_cubie_a7z.sh shreyash@<ip>:~/
#   5. Run: chmod +x ~/post_flash_setup_v5_cubie_a7z.sh
#          sudo ~/post_flash_setup_v5_cubie_a7z.sh
# =====================================================

set -e

if [ "$EUID" -ne 0 ]; then
  echo "Please run as root (sudo)"
  exit 1
fi

echo "================================================"
echo " RADXA CUBIE A7Z - POST-FLASH SETUP v5.0"
echo " (Debian 11 CLI + SD Card)"
echo "================================================"
echo ""

USER_NAME="shreyash"
USER_HOME="/home/$USER_NAME"
DRONE_DIR="$USER_HOME/drone_project"
BRIDGE_DIR="$DRONE_DIR/raxda_bridge"

# ===== 0. CREATE USER + CONNECT WIFI + ENABLE SSH (FIRST!) =====
echo "[0/16] Setting up user, WiFi, and SSH immediately..."

# Create user shreyash if not exists
if ! id "shreyash" &>/dev/null; then
    adduser --disabled-password --gecos "" shreyash 2>/dev/null || true
    echo "shreyash:1" | chpasswd
    usermod -aG sudo shreyash
    echo "  User shreyash created (password: 1)"
else
    echo "  User shreyash already exists"
fi

# Connect to phone hotspot WiFi
nmcli dev wifi connect "S21 ultra" password "22219413" 2>/dev/null || echo "  WiFi connect failed (may already be connected)"
echo "  WiFi: S21 ultra connected"

# Force enable SSH NOW (so we never get locked out)
apt-get install -y openssh-server 2>/dev/null || true
systemctl enable ssh 2>/dev/null || systemctl enable sshd 2>/dev/null || true
systemctl start ssh 2>/dev/null || systemctl start sshd 2>/dev/null || true
sed -i 's/#PasswordAuthentication.*/PasswordAuthentication yes/' /etc/ssh/sshd_config 2>/dev/null || true
sed -i 's/PasswordAuthentication no/PasswordAuthentication yes/' /etc/ssh/sshd_config 2>/dev/null || true
systemctl restart ssh 2>/dev/null || systemctl restart sshd 2>/dev/null || true
echo "  SSH ENABLED"
echo ""
echo "  >>> You can now SSH from laptop: ssh shreyash@$(hostname -I | awk '{print $1}') <<<"
echo ""

# ===== 1. SYSTEM PACKAGES =====
echo "[1/16] Installing system packages..."
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

# ===== 2. PYTHON PACKAGES =====
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
    lxml \
    fastcrc \
    || echo "  Some pip packages may have failed"
echo "  Done"

# ===== 2.5. DISK CLEANUP (Free space after installs) =====
echo ""
echo "[2.5/14] Cleaning up to free disk space..."
apt-get clean
apt-get autoremove -y 2>/dev/null || true
rm -rf /tmp/pip-* /tmp/pip-req-build-* 2>/dev/null || true
rm -rf /var/cache/apt/archives/*.deb 2>/dev/null || true
rm -rf /root/.cache/pip 2>/dev/null || true
echo "  Done ($(df -h / | tail -1 | awk '{print $4}') free)"

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
echo "  >>> AFTER SETUP: Run 'sudo tailscale up' to authenticate <<<"
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

# ===== 5. ENABLE UART0 FOR FC (Pin 8/10) =====
echo ""
echo "[5/14] Enabling UART0 overlay for FC communication..."
# On Cubie A7Z, Pin 8 = UART0-TX (PB9), Pin 10 = UART0-RX (PB10)
# Use rsetup if available, otherwise configure manually
if command -v rsetup &> /dev/null; then
    echo "  rsetup found — enable UART0 overlay via: sudo rsetup"
    echo "  Go to: Overlay → Manage overlays → Enable UART0"
    echo "  >>> YOU MUST DO THIS MANUALLY AFTER SETUP <<<"
else
    # Manual device tree overlay approach
    echo "  rsetup not found — attempting manual UART0 enable"
    # The exact overlay name depends on the Radxa Debian image
    # Common: radxa-cubie-a7z-uart0
    if [ -d "/boot/dtbs/overlays" ]; then
        echo "  Check /boot/dtbs/overlays/ for UART0 overlay"
    fi
fi
echo "  IMPORTANT: FC UART is /dev/ttyS0 on Cubie A7Z"

# ===== 6. SERIAL PORT PERMISSIONS =====
echo ""
echo "[6/14] Setting permissions..."
usermod -a -G dialout $USER_NAME 2>/dev/null || true
usermod -a -G video $USER_NAME 2>/dev/null || true
usermod -a -G bluetooth $USER_NAME 2>/dev/null || true
usermod -a -G sudo $USER_NAME 2>/dev/null || true
usermod -a -G i2c $USER_NAME 2>/dev/null || true

cat > /etc/udev/rules.d/99-drone-serial.rules << 'UDEV_EOF'
# FC on UART0 (Cubie A7Z Pin 8/10) — Allwinner uses ttyAS
KERNEL=="ttyAS0", MODE="0666"
KERNEL=="ttyAS1", MODE="0666"
KERNEL=="ttyS0", MODE="0666"
# YDLidar on USB
KERNEL=="ttyUSB*", MODE="0666"
# GoPro USB webcam / Camera
KERNEL=="video*", MODE="0666"
UDEV_EOF

# Quiet kernel console (prevent interference with FC serial reads)
echo "kernel.printk = 0 0 0 0" > /etc/sysctl.d/99-quiet-console.conf
sysctl -p /etc/sysctl.d/99-quiet-console.conf 2>/dev/null || true

udevadm control --reload-rules 2>/dev/null || true
echo "  Done"

# ===== 7. THERMAL PROTECTION (NEVER SHUTDOWN) =====
echo ""
echo "[7/14] Setting up thermal protection..."

cat > /usr/local/bin/thermal_guard.sh << 'THERMAL_EOF'
#!/bin/bash
# Thermal guard — CPU throttle only, NEVER shutdown (drone safe)
# A733 runs cooler than RK3566 but still protect
THROTTLE_TEMP=65000   # 65C start throttling (A733 has more headroom)
MAX_TEMP=75000        # 75C aggressive throttle
NORMAL_FREQ=2000000   # 2.0 GHz (A733 max)
THROTTLE_FREQ=1600000 # 1.6 GHz
LOW_FREQ=1000000      # 1.0 GHz emergency

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
echo "  Thermal guard active (throttle 65C, aggressive 75C, NEVER shutdown)"

# ===== 8. HARDWARE WATCHDOG =====
echo ""
echo "[8/14] Enabling hardware watchdog..."
sed -i 's/#RuntimeWatchdogSec=.*/RuntimeWatchdogSec=15/' /etc/systemd/system.conf 2>/dev/null || true
sed -i 's/#RebootWatchdogSec=.*/RebootWatchdogSec=2min/' /etc/systemd/system.conf 2>/dev/null || true
echo "softdog" > /etc/modules-load.d/watchdog.conf 2>/dev/null || true
echo "  Done (auto-reboot on 15s freeze)"

# ===== 9. KERNEL PANIC AUTO-RECOVERY =====
echo ""
echo "[9/14] Enabling kernel panic auto-recovery..."
cat > /etc/sysctl.d/99-drone-panic.conf << 'PANIC_EOF'
kernel.panic = 10
kernel.sysrq = 1
PANIC_EOF
sysctl --system 2>/dev/null || true
echo "  Done (auto-reboot 10s after panic)"

# ===== 10. FILESYSTEM HARDENING (prevent corruption from power loss) =====
echo ""
echo "[10/16] Hardening filesystem against power loss..."
# Use journal_data mode (safer than writeback for sudden power loss)
tune2fs -o journal_data $(findmnt -n -o SOURCE /) 2>/dev/null || true
# Mount with sync option to flush writes immediately (ONLY for ext4, not FAT32)
ROOT_DEV=$(findmnt -n -o SOURCE /)
if [ -n "$ROOT_DEV" ]; then
    # Only add sync to the root ext4 partition, not FAT32 boot partitions
    sed -i "\|$ROOT_DEV|s/defaults/defaults,sync/" /etc/fstab 2>/dev/null || true
fi
# Reduce journal writes — use tmpfs for logs to prevent SD card corruption
echo "tmpfs /tmp tmpfs defaults,noatime,nosuid,nodev,size=100m 0 0" >> /etc/fstab 2>/dev/null || true
echo "tmpfs /var/log tmpfs defaults,noatime,nosuid,nodev,size=50m 0 0" >> /etc/fstab 2>/dev/null || true
# Add safe shutdown alias
echo 'alias off="sudo sync && sudo poweroff"' >> /home/$USER_NAME/.bashrc 2>/dev/null || true
echo "  Done (journal_data mode, sync mount, tmpfs for logs)"

# ===== 11. CAMERA SETUP SCRIPT =====
echo ""
echo "[11/14] Creating camera setup script..."
cat > $USER_HOME/setup_camera.sh << 'CAMERA_EOF'
#!/bin/bash
# Camera setup for Cubie A7Z — Pi Camera V2 (IMX219)
# A7Z has 1x 4-lane or 2x 2-lane CSI

# Check for camera device
if [ -e /dev/video0 ]; then
    echo "Camera device found at /dev/video0"
    v4l2-ctl -d /dev/video0 --list-formats-ext 2>/dev/null | head -20
else
    echo "No camera device found — check CSI connection"
fi

# Try media-ctl setup for IMX219
if [ -e /dev/media0 ]; then
    sudo media-ctl -r -d /dev/media0 2>/dev/null || true
    sudo v4l2-ctl -d /dev/video0 --set-fmt-video=width=1920,height=1080,pixelformat=NV12 2>/dev/null || true
    echo "Camera: IMX219 configured at 1920x1080"
fi
CAMERA_EOF
chmod +x $USER_HOME/setup_camera.sh
chown $USER_NAME:$USER_NAME $USER_HOME/setup_camera.sh

# ===== 12. DRONE BRIDGE LAUNCHER =====
echo ""
echo "[12/14] Creating drone bridge launcher..."

cat > $USER_HOME/launch_bridge.sh << 'LAUNCH_EOF'
#!/bin/bash
echo "==============================="
echo " CUBIE A7Z DRONE BRIDGE"
echo "==============================="

# Step 1: Camera
echo "[1/3] Configuring Camera..."
~/setup_camera.sh 2>/dev/null || echo "Camera setup skipped"

# Step 2: Tailscale
echo "[2/3] Checking Tailscale..."
tailscale status 2>/dev/null | head -3 || echo "Tailscale not running"

# Step 3: Bridge
echo "[3/3] Launching Bridge..."

# Stop UART console so bridge can use ttyAS0 for FC
# When bridge stops, getty auto-restarts (trap EXIT)
sudo systemctl stop serial-getty@ttyAS0.service 2>/dev/null
sudo chmod 666 /dev/ttyAS0 /dev/ttyAS1 2>/dev/null
trap "sudo systemctl start serial-getty@ttyAS0.service" EXIT

# Kill stale port locks
sudo fuser -k /dev/ttyAS0 8888/udp 8000/tcp 8080/tcp 2>/dev/null || true
sleep 0.5

BRIDGE="$HOME/drone_project/raxda_bridge/real_bridge_service.py"
if [ -f "$BRIDGE" ]; then
    cd "$(dirname "$BRIDGE")"
    exec python3 "$BRIDGE"
else
    echo "ERROR: Bridge code not found at $BRIDGE"
    echo "SCP your code: scp -r raxda_bridge/ shreyash@<ip>:~/drone_project/"
    exit 1
fi
LAUNCH_EOF
chmod +x $USER_HOME/launch_bridge.sh
chown $USER_NAME:$USER_NAME $USER_HOME/launch_bridge.sh

# ===== 13. SYSTEMD SERVICE =====
echo ""
echo "[13/14] Creating drone bridge service..."

cat > /etc/systemd/system/drone-bridge.service << SERVICE_EOF
[Unit]
Description=Cubie A7Z Drone Bridge Service
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

# ===== 14. ENABLE OVERLAYS (Camera + UART4) =====
echo ""
echo "[14/16] Enabling hardware overlays..."
# Enable Camera 8M 219 (IMX219 / Pi Camera V2) and UART4 overlays
# These are normally done via rsetup but we automate it
OVERLAY_DIR="/boot/dtbs/overlays"
if [ -d "$OVERLAY_DIR" ]; then
    # Try to find and enable camera and UART overlays
    OVERLAY_CONF="/boot/uEnv.txt"
    if [ -f "$OVERLAY_CONF" ]; then
        # Add overlays to boot config if not already present
        if ! grep -q "camera-8m-219" "$OVERLAY_CONF" 2>/dev/null; then
            echo "# Camera IMX219 overlay" >> "$OVERLAY_CONF"
            echo "overlays=radxa-cubie-a7z-camera-8m-219 radxa-cubie-a7z-uart4" >> "$OVERLAY_CONF" 2>/dev/null || true
            echo "  Added camera + UART4 overlays to boot config"
        fi
    fi
    # Also try rsetup non-interactive method
    if command -v rsetup &> /dev/null; then
        echo "  rsetup available — overlays may need manual enable via: sudo rsetup"
    fi
else
    echo "  Overlay directory not found — overlays may need manual configuration"
fi
echo "  Done"

# ===== 15. CREATE USER (if not exists) =====
echo ""
echo "[15/16] Creating user $USER_NAME..."
if id "$USER_NAME" &>/dev/null; then
    echo "  User $USER_NAME already exists"
else
    adduser --disabled-password --gecos "" $USER_NAME 2>/dev/null || true
    echo "$USER_NAME:1" | chpasswd
    echo "  User $USER_NAME created with password: 1"
fi
usermod -aG sudo $USER_NAME 2>/dev/null || true

# ===== 16. SSH & SUDO =====
echo ""
echo "[16/16] Configuring SSH and sudo..."
# FORCE enable SSH — this is critical, without it we're locked out
apt-get install -y openssh-server 2>/dev/null || true
systemctl enable ssh 2>/dev/null || systemctl enable sshd 2>/dev/null || true
systemctl start ssh 2>/dev/null || systemctl start sshd 2>/dev/null || true
# Allow password login (needed for first connection)
sed -i 's/#PasswordAuthentication.*/PasswordAuthentication yes/' /etc/ssh/sshd_config 2>/dev/null || true
sed -i 's/PasswordAuthentication no/PasswordAuthentication yes/' /etc/ssh/sshd_config 2>/dev/null || true
systemctl restart ssh 2>/dev/null || systemctl restart sshd 2>/dev/null || true
echo "  SSH FORCE ENABLED"

cat > /etc/sudoers.d/drone-ops << 'SUDOERS_EOF'
shreyash ALL=(ALL) NOPASSWD: /usr/bin/mount, /usr/bin/umount, /usr/bin/systemctl restart drone-bridge, /usr/bin/systemctl stop drone-bridge, /usr/bin/systemctl start drone-bridge, /usr/bin/fuser, /usr/sbin/fsck, /usr/sbin/fsck.ext4
SUDOERS_EOF
chmod 440 /etc/sudoers.d/drone-ops

# [15/15] WiFi Hotspot for ESP32 (4G dongle handles internet, WiFi chip runs AP mode)
echo ""
echo "[15/15] Creating WiFi hotspot for ESP32 connection..."
# Create a persistent WiFi hotspot that auto-starts on boot
# ESP32 connects to this hotspot for sensor/gimbal UDP communication
nmcli connection add type wifi ifname wlan0 con-name DroneAP autoconnect yes \
    ssid "DroneAP" \
    wifi.mode ap \
    wifi.band bg \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk "dronepass123" \
    ipv4.method shared \
    ipv4.addresses 10.42.0.1/24 \
    2>/dev/null || echo "⚠️ Hotspot creation failed — configure manually: nmcli dev wifi hotspot ssid DroneAP password dronepass123"

# Enable auto-start of hotspot
nmcli connection modify DroneAP connection.autoconnect yes 2>/dev/null || true
echo "✅ WiFi Hotspot 'DroneAP' created (password: dronepass123, IP: 10.42.0.1)"
echo "   ESP32 firmware must use: ssid='DroneAP', password='dronepass123', radxa_ip=10.42.0.1"

# Create drone project directory
mkdir -p "$BRIDGE_DIR"
chown -R $USER_NAME:$USER_NAME "$DRONE_DIR"

# Auto-fix UART port names for Allwinner A733 (ttyAS vs ttyS)
# This runs after bridge code is SCP'd — safe to run even if files don't exist yet
if [ -f "$BRIDGE_DIR/real_bridge_service.py" ]; then
    sed -i 's|"/dev/ttyS0"|"/dev/ttyAS0"|g' "$BRIDGE_DIR/real_bridge_service.py"
    sed -i 's|"/dev/ttyS2"|"/dev/ttyAS1"|g' "$BRIDGE_DIR/real_bridge_service.py"
    echo "  Bridge UART ports updated for Allwinner (ttyAS0/ttyAS1)"
fi

echo ""
echo "================================================"
echo " POST-FLASH SETUP v5.0 COMPLETE!"
echo " (Radxa Cubie A7Z + Debian 11)"
echo "================================================"
echo ""
echo " Hardware Ports:"
echo "   FC:        /dev/ttyS0 @ 57600 baud (UART0 Pin 8/10)"
echo "   LiDAR:     /dev/ttyUSB0 (via USB hub)"
echo "   Camera:    /dev/video0 (IMX219 Pi Camera V2)"
echo "   ESP32:     WiFi UDP port 8888"
echo "   GoPro:     BLE + WiFi (via AIC8800)"
echo ""
echo " Protection:"
echo "   Thermal:   Throttle at 65C, aggressive 75C (NEVER shutdown)"
echo "   Watchdog:  Auto-reboot on 15s freeze"
echo "   Panic:     Auto-reboot 10s after kernel panic"
echo "   Journal:   Filesystem write protection enabled"
echo ""
echo " Services:"
echo "   drone-bridge:     enabled (auto-start on boot)"
echo "   thermal-guard:    enabled (CPU throttle)"
echo "   tailscaled:       enabled"
echo ""
echo " ⚠️  DO THESE MANUALLY:"
echo "   1. sudo tailscale up (authenticate)"
echo "   2. SCP bridge code:"
echo "      scp -r raxda_bridge/ shreyash@<ip>:~/drone_project/"
echo "      scp -r raxda/ shreyash@<ip>:~/drone_project/"
echo "   3. sudo reboot"
echo ""
echo " ⚠️  NEVER PULL POWER WITHOUT SHUTDOWN!"
echo "   Type 'off' or 'sudo poweroff' before disconnecting battery"
echo ""
echo " Tailscale IPs:"
echo "   Laptop: 100.64.0.10"
echo "   Phone:  100.64.0.20"
echo ""
echo " Server:"
echo "   wss://drone-server-r0qe.onrender.com/ws/connect/RADXA_X"
echo ""
echo "================================================"
