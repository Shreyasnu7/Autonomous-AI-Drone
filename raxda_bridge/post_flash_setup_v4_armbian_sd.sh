#!/bin/bash
# =====================================================
# RADXA ZERO 3W - POST-FLASH SETUP v4.0 (ARMBIAN MINIMAL + SD CARD)
# THE DEFINITIVE DRONE SETUP SCRIPT
# =====================================================
# Combines: v2 (Armbian overlays, camera pipeline) + v3 (thermal guard,
# watchdog, filesystem hardening, Tailscale, full packages)
#
# For: Armbian Minimal (Bookworm/Trixie) booting from SD card ONLY
# No eMMC dependency. No GUI. Drone-optimized.
#
# Usage:
#   1. Flash Armbian Minimal to SD card
#   2. Boot, login as root (default pw: 1234), create user shreyash
#   3. Connect WiFi: nmcli dev wifi connect "SSID" password "PASS"
#   4. From laptop:
#      scp post_flash_setup_v4_armbian_sd.sh shreyash@<ip>:~/
#      ssh shreyash@<ip>
#      chmod +x ~/post_flash_setup_v4_armbian_sd.sh
#      sudo ~/post_flash_setup_v4_armbian_sd.sh
# =====================================================

set -e

if [ "$EUID" -ne 0 ]; then
  echo "Please run as root (sudo)"
  exit 1
fi

echo "================================================"
echo " RADXA ZERO 3W - POST-FLASH SETUP v4.0"
echo " (Armbian Minimal + SD Card Boot)"
echo "================================================"
echo ""

USER_NAME="shreyash"
USER_HOME="/home/$USER_NAME"
DRONE_DIR="$USER_HOME/drone_project"
BRIDGE_DIR="$DRONE_DIR/raxda_bridge"

# ===== 1. SYSTEM PACKAGES =====
echo "[1/15] Installing system packages..."
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
echo "[2/15] Installing Python packages..."
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

# ===== 3. TAILSCALE VPN =====
echo ""
echo "[3/15] Installing Tailscale..."
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
echo "[4/15] Configuring USB serial drivers..."
modprobe cp210x 2>/dev/null || true
modprobe ch341 2>/dev/null || true
for mod in cp210x ch341; do
    if ! grep -q "$mod" /etc/modules 2>/dev/null; then
        echo "$mod" >> /etc/modules
    fi
done
echo "  Done"

# ===== 5. ARMBIAN OVERLAYS (UART + CAMERA) =====
echo ""
echo "[5/15] Configuring Armbian overlays..."

TARGET_ENV=""
for f in /boot/armbianEnv.txt /boot/uEnv.txt; do
    if [ -f "$f" ]; then
        TARGET_ENV="$f"
        break
    fi
done

if [ -n "$TARGET_ENV" ]; then
    cp "$TARGET_ENV" "${TARGET_ENV}.bak" 2>/dev/null || true

    # Add stmmac blacklist (prevents crash on some boards)
    if ! grep -q "initcall_blacklist" "$TARGET_ENV"; then
        if grep -q "^extraargs=" "$TARGET_ENV"; then
            # Append to existing extraargs
            sed -i '/^extraargs=/ s/$/ initcall_blacklist=stmmac_pltfr_driver_init,rk_gmac_dwmac_driver_init/' "$TARGET_ENV"
        else
            echo "extraargs=initcall_blacklist=stmmac_pltfr_driver_init,rk_gmac_dwmac_driver_init" >> "$TARGET_ENV"
        fi
        echo "  Added stmmac blacklist to extraargs"
    fi

    # Required overlays: uart2 (FC), imx219 camera
    for overlay in uart2 rk3566-radxa-zero3-imx219; do
        if grep -q "^overlays=" "$TARGET_ENV"; then
            if ! grep -q "$overlay" "$TARGET_ENV"; then
                sed -i "/^overlays=/ s/$/ $overlay/" "$TARGET_ENV"
                echo "  Added overlay: $overlay"
            fi
        else
            echo "overlays=$overlay" >> "$TARGET_ENV"
        fi
    done
    echo "  Overlays configured (reboot required)"
else
    echo "  WARNING: No boot env file found!"
fi

# ===== 6. SERIAL PORT PERMISSIONS =====
echo ""
echo "[6/15] Setting permissions..."
usermod -a -G dialout $USER_NAME 2>/dev/null || true
usermod -a -G video $USER_NAME 2>/dev/null || true
usermod -a -G bluetooth $USER_NAME 2>/dev/null || true
usermod -a -G sudo $USER_NAME 2>/dev/null || true
usermod -a -G i2c $USER_NAME 2>/dev/null || true

cat > /etc/udev/rules.d/99-drone-serial.rules << 'UDEV_EOF'
# FC on UART2
KERNEL=="ttyS2", MODE="0666"
# YDLidar on USB
KERNEL=="ttyUSB*", MODE="0666"
# GoPro USB webcam
KERNEL=="video*", MODE="0666"
UDEV_EOF

udevadm control --reload-rules 2>/dev/null || true
echo "  Done"

# ===== 7. THERMAL PROTECTION (NEVER SHUTDOWN — DRONE SAFE) =====
echo ""
echo "[7/15] Setting up thermal protection..."

cat > /usr/local/bin/thermal_guard.sh << 'THERMAL_EOF'
#!/bin/bash
# Thermal guard — CPU throttle only, NEVER shutdown (drone safe)
THROTTLE_TEMP=60000
MAX_TEMP=70000
NORMAL_FREQ=1416000
THROTTLE_FREQ=1200000
LOW_FREQ=816000

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

# ===== 8. HARDWARE WATCHDOG =====
echo ""
echo "[8/15] Enabling hardware watchdog..."
sed -i 's/#RuntimeWatchdogSec=.*/RuntimeWatchdogSec=15/' /etc/systemd/system.conf 2>/dev/null || true
sed -i 's/#RebootWatchdogSec=.*/RebootWatchdogSec=2min/' /etc/systemd/system.conf 2>/dev/null || true
echo "softdog" > /etc/modules-load.d/watchdog.conf 2>/dev/null || true
echo "  Done (auto-reboot on 15s freeze)"

# ===== 9. KERNEL PANIC AUTO-RECOVERY =====
echo ""
echo "[9/15] Enabling kernel panic auto-recovery..."
cat > /etc/sysctl.d/99-drone-panic.conf << 'PANIC_EOF'
kernel.panic = 10
kernel.sysrq = 1
PANIC_EOF
sysctl --system 2>/dev/null || true
echo "  Done (auto-reboot 10s after panic)"

# ===== 10. FILESYSTEM HARDENING =====
echo ""
echo "[10/15] Hardening filesystem..."
tune2fs -o journal_data_writeback $(findmnt -n -o SOURCE /) 2>/dev/null || true
echo "  Done"

# ===== 11. CAMERA SETUP SCRIPT =====
echo ""
echo "[11/15] Creating camera setup script..."
cat > $USER_HOME/setup_camera.sh << 'CAMERA_EOF'
#!/bin/bash
# Camera pipeline for Pi Camera V2.1 (IMX219) + GoPro

# IMX219 via CSI
if [ -e /dev/media0 ]; then
    sudo media-ctl -r -d /dev/media0 2>/dev/null || true

    SENSOR="'m00_b_imx219 2-0010'"
    DPHY="'rockchip-csi2-dphy0'"
    CSI="'rkisp-csi-subdev'"
    ISP="'rkisp-isp-subdev'"
    MAIN="'rkisp_mainpath'"

    sudo media-ctl -d /dev/media0 -l "$SENSOR:0->$DPHY:0 [1]" 2>/dev/null || true
    sudo media-ctl -d /dev/media0 -l "$DPHY:1->$CSI:0 [1]" 2>/dev/null || true
    sudo media-ctl -d /dev/media0 -l "$CSI:1->$ISP:0 [1]" 2>/dev/null || true
    sudo media-ctl -d /dev/media0 -l "$ISP:2->$MAIN:0 [1]" 2>/dev/null || true

    sudo media-ctl -d /dev/media0 -V "$SENSOR:0 [fmt:SRGGB10_1X10/1920x1080]" 2>/dev/null || true
    sudo media-ctl -d /dev/media0 -V "$ISP:2 [fmt:YUYV8_2X8/1920x1080]" 2>/dev/null || true

    sudo v4l2-ctl -d /dev/video0 --set-fmt-video=width=1920,height=1080,pixelformat=NV12 2>/dev/null || true
    echo "Camera: IMX219 configured at 1920x1080"
fi

# GoPro USB webcam check
for dev in /dev/video*; do
    if v4l2-ctl -d "$dev" --info 2>/dev/null | grep -qi "gopro"; then
        echo "Camera: GoPro USB webcam found at $dev"
        break
    fi
done
CAMERA_EOF
chmod +x $USER_HOME/setup_camera.sh
chown $USER_NAME:$USER_NAME $USER_HOME/setup_camera.sh

# ===== 12. DRONE BRIDGE LAUNCHER =====
echo ""
echo "[12/15] Creating drone bridge launcher..."

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

# Kill stale UART locks
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

# ===== 13. SYSTEMD SERVICE =====
echo ""
echo "[13/15] Creating drone bridge service..."

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
Environment=TS_LAPTOP_IP=100.84.75.22
Environment=TS_PHONE_IP=100.109.112.110

[Install]
WantedBy=multi-user.target
SERVICE_EOF

systemctl daemon-reload
systemctl enable drone-bridge
echo "  Done (auto-starts on boot)"

# ===== 14. SSH & SUDO =====
echo ""
echo "[14/15] Configuring SSH and sudo..."
systemctl enable ssh 2>/dev/null || systemctl enable sshd 2>/dev/null || true

cat > /etc/sudoers.d/drone-ops << 'SUDOERS_EOF'
shreyash ALL=(ALL) NOPASSWD: /usr/bin/mount, /usr/bin/umount, /usr/bin/systemctl restart drone-bridge, /usr/bin/systemctl stop drone-bridge, /usr/bin/systemctl start drone-bridge, /usr/bin/fuser, /usr/sbin/fsck, /usr/sbin/fsck.ext4
SUDOERS_EOF
chmod 440 /etc/sudoers.d/drone-ops
echo "  Done"

# ===== 15. FINAL SETUP =====
echo ""
echo "[15/15] Final setup..."

# Create drone project directory
mkdir -p "$BRIDGE_DIR"
chown -R $USER_NAME:$USER_NAME "$DRONE_DIR"

echo ""
echo "================================================"
echo " POST-FLASH SETUP v4.0 COMPLETE!"
echo " (Armbian Minimal + SD Card)"
echo "================================================"
echo ""
echo " Hardware Ports:"
echo "   FC:        /dev/ttyS2 @ 57600 baud"
echo "   LiDAR:     /dev/ttyUSB0 (via USB hub)"
echo "   Camera:    /dev/video0 (IMX219 1080p / GoPro)"
echo "   ESP32:     WiFi UDP port 8888"
echo ""
echo " Protection:"
echo "   Thermal:   Throttle at 60C, aggressive 70C (NEVER shutdown)"
echo "   Watchdog:  Auto-reboot on 15s freeze"
echo "   Panic:     Auto-reboot 10s after kernel panic"
echo "   Journal:   Filesystem write protection enabled"
echo "   Stmmac:    Ethernet driver blacklisted (prevents boot crash)"
echo ""
echo " Services:"
echo "   drone-bridge:     enabled (auto-start on boot)"
echo "   thermal-guard:    enabled (CPU throttle)"
echo "   tailscaled:       enabled"
echo ""
echo " NEXT STEPS:"
echo "   1. sudo tailscale up              (authenticate)"
echo "   2. SCP bridge code:"
echo "      scp -r raxda_bridge/ shreyash@<ip>:~/drone_project/"
echo "   3. sudo reboot                    (apply overlays)"
echo "   4. After reboot, bridge auto-starts"
echo ""
echo " Tailscale IPs:"
echo "   Laptop: 100.84.75.22"
echo "   Phone:  100.109.112.110"
echo ""
echo " Server:"
echo "   wss://drone-server-r0qe.onrender.com/ws/connect/RADXA_X"
echo ""
echo "================================================"
