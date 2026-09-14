#!/bin/bash
# =====================================================
# RADXA ZERO 3W - POST-FLASH SETUP v2.0
# =====================================================
# Run this after fresh Armbian Minimal flash to restore
# ALL settings. Drone code lives on SD card (read-only).
#
# Usage:
#   1. Flash Armbian to eMMC via RKDevTool
#   2. Boot, login as root (default pw: 1234)
#   3. Create user: adduser shreyash (add to sudo)
#   4. SCP this script: scp post_flash_setup_v2.sh shreyash@<ip>:~/
#   5. Run: chmod +x ~/post_flash_setup_v2.sh && sudo ~/post_flash_setup_v2.sh
# =====================================================

set -e

echo "========================================"
echo " RADXA ZERO 3W - POST-FLASH SETUP v2.0"
echo "========================================"
echo ""

USER_NAME="shreyash"

# ===== 1. MOUNT SD CARD =====
echo "[1/12] Mounting SD card..."
mkdir -p /mnt/sdcard

# Try both common SD devices
if ! grep -qs '/mnt/sdcard' /proc/mounts; then
    mount /dev/mmcblk1p1 /mnt/sdcard 2>/dev/null || \
    mount /dev/mmcblk0p1 /mnt/sdcard 2>/dev/null || \
    echo "WARNING: SD card not found. Plug it in and re-run."
fi

# Add to fstab for auto-mount (read-only for protection)
if ! grep -q '/mnt/sdcard' /etc/fstab; then
    echo "/dev/mmcblk1p1 /mnt/sdcard ext4 ro,noatime,errors=remount-ro 0 2" >> /etc/fstab
    echo "  Added SD card to fstab (read-only)"
fi

chown -R $USER_NAME:$USER_NAME /mnt/sdcard 2>/dev/null || true
echo "  SD card at /mnt/sdcard"

# ===== 2. SYSTEM PACKAGES =====
echo ""
echo "[2/12] Installing system packages..."
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
    python3-gst-1.0 \
    net-tools \
    wireless-tools \
    network-manager \
    bluez \
    bluetooth \
    curl \
    || echo "  Some packages may have failed (non-critical)"
echo "  System packages installed"

# ===== 3. PYTHON PACKAGES (SYSTEM-WIDE) =====
echo ""
echo "[3/12] Installing Python packages (system-wide)..."
# Using system python (no venv) — simpler, works even when SD is read-only
pip3 install --break-system-packages \
    pymavlink \
    pyserial \
    websockets \
    aiohttp \
    numpy \
    smbus2 \
    bleak \
    || echo "  Some pip packages may have failed"
echo "  Python packages installed"

# ===== 4. USB-TO-SERIAL DRIVERS =====
echo ""
echo "[4/12] Configuring USB serial drivers..."
modprobe cp210x 2>/dev/null || true
modprobe ch341 2>/dev/null || true
# Persist across reboots
for mod in cp210x ch341; do
    if ! grep -q "$mod" /etc/modules 2>/dev/null; then
        echo "$mod" >> /etc/modules
    fi
done
echo "  USB serial drivers configured"

# ===== 5. SERIAL PORT OVERLAYS =====
echo ""
echo "[5/12] Configuring serial port overlays..."

TARGET_ENV=""
for f in /boot/armbianEnv.txt /boot/uEnv.txt; do
    if [ -f "$f" ]; then
        TARGET_ENV="$f"
        break
    fi
done

if [ -n "$TARGET_ENV" ]; then
    cp "$TARGET_ENV" "${TARGET_ENV}.bak" 2>/dev/null || true

    # Required overlays: uart2 (FC), uart4 (ESP32), i2c3-m0, imx219 camera
    for overlay in uart2 uart4 radxa-zero3-imx219; do
        if grep -q "overlays=" "$TARGET_ENV"; then
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
    echo "  WARNING: No boot env file found! Use rsetup to enable overlays."
fi

# ===== 6. SERIAL PORT PERMISSIONS =====
echo ""
echo "[6/12] Setting serial port permissions..."
usermod -a -G dialout $USER_NAME
usermod -a -G video $USER_NAME
usermod -a -G bluetooth $USER_NAME
usermod -a -G sudo $USER_NAME

# Persistent udev rules for serial ports
cat > /etc/udev/rules.d/99-drone-serial.rules << 'UDEV_EOF'
# FC on UART2
KERNEL=="ttyS2", MODE="0666"
# ESP32 on UART4
KERNEL=="ttyS4", MODE="0666"
# YDLidar on USB
KERNEL=="ttyUSB*", MODE="0666"
UDEV_EOF

udevadm control --reload-rules
echo "  Serial permissions configured"

# ===== 7. PASSWORDLESS SUDO FOR MOUNT =====
echo ""
echo "[7/12] Setting up passwordless mount (for remote deploys)..."
cat > /etc/sudoers.d/sdcard-deploy << 'SUDOERS_EOF'
# Allow mount/umount without password for SD card deploy cycle
shreyash ALL=(ALL) NOPASSWD: /usr/bin/mount, /usr/bin/umount, /usr/sbin/fsck, /usr/sbin/fsck.ext4
SUDOERS_EOF
chmod 440 /etc/sudoers.d/sdcard-deploy
echo "  Passwordless mount configured"

# ===== 8. TAILSCALE VPN =====
echo ""
echo "[8/12] Installing Tailscale..."
if ! command -v tailscale &> /dev/null; then
    curl -fsSL https://tailscale.com/install.sh | sh
    echo "  Tailscale installed"
else
    echo "  Tailscale already installed"
fi
systemctl enable --now tailscaled
echo "  Tailscale daemon enabled"
echo ""
echo "  >>> IMPORTANT: Run 'sudo tailscale up' after setup to authenticate <<<"
echo "  >>> Then verify: tailscale ip -4 (should show 100.94.63.59) <<<"
echo ""

# ===== 9. THERMAL WATCHDOG =====
echo ""
echo "[9/12] Setting up thermal protection..."
cat > /usr/local/bin/thermal_watchdog.sh << 'THERMAL_EOF'
#!/bin/bash
# Thermal watchdog - prevents eMMC corruption from overheating
CRITICAL_TEMP=80000  # 80°C in millidegrees

while true; do
    TEMP=$(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null || echo "0")
    if [ "$TEMP" -gt "$CRITICAL_TEMP" ]; then
        echo "THERMAL CRITICAL: ${TEMP}mC - Shutting down to protect hardware!"
        logger "THERMAL WATCHDOG: Emergency shutdown at ${TEMP}mC"
        shutdown -h now "Thermal emergency shutdown"
    fi
    sleep 10
done
THERMAL_EOF
chmod +x /usr/local/bin/thermal_watchdog.sh

# Systemd service for thermal watchdog
cat > /etc/systemd/system/thermal-watchdog.service << 'TWD_EOF'
[Unit]
Description=Thermal Watchdog (Prevents eMMC Corruption)
After=multi-user.target

[Service]
Type=simple
ExecStart=/usr/local/bin/thermal_watchdog.sh
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
TWD_EOF

systemctl daemon-reload
systemctl enable thermal-watchdog
systemctl start thermal-watchdog
echo "  Thermal watchdog active (auto-shutdown at 80C)"

# ===== 10. CAMERA SETUP SCRIPT =====
echo ""
echo "[10/12] Creating camera setup script..."
cat > /home/$USER_NAME/setup_camera.sh << 'CAMERA_EOF'
#!/bin/bash
# Camera pipeline configuration for Pi Camera V2.1 (IMX219)
set -e

sudo media-ctl -r -d /dev/media0 2>/dev/null || true

SENSOR="'m00_b_imx219 2-0010'"
DPHY="'rockchip-csi2-dphy0'"
CSI="'rkisp-csi-subdev'"
ISP="'rkisp-isp-subdev'"
MAIN="'rkisp_mainpath'"

sudo media-ctl -d /dev/media0 -l "$SENSOR:0->$DPHY:0 [1]"
sudo media-ctl -d /dev/media0 -l "$DPHY:1->$CSI:0 [1]"
sudo media-ctl -d /dev/media0 -l "$CSI:1->$ISP:0 [1]"
sudo media-ctl -d /dev/media0 -l "$ISP:2->$MAIN:0 [1]"

sudo media-ctl -d /dev/media0 -V "$SENSOR:0 [fmt:SRGGB10_1X10/1920x1080]"
sudo media-ctl -d /dev/media0 -V "$ISP:2 [fmt:YUYV8_2X8/1920x1080]"

sudo v4l2-ctl -d /dev/video0 --set-fmt-video=width=1920,height=1080,pixelformat=NV12

echo "Camera IMX219 configured at 1920x1080"
CAMERA_EOF
chmod +x /home/$USER_NAME/setup_camera.sh
chown $USER_NAME:$USER_NAME /home/$USER_NAME/setup_camera.sh
echo "  Camera setup script created"

# ===== 11. DRONE BRIDGE SERVICE =====
echo ""
echo "[11/12] Creating drone bridge service..."

# Create launcher that uses system python + persistent dir fallback
cat > /home/$USER_NAME/launch_bridge.sh << 'LAUNCH_EOF'
#!/bin/bash
echo "RADXA DRONE BRIDGE LAUNCHER"
echo "============================"

# Step 1: Camera
echo "Step 1: Configuring Camera..."
~/setup_camera.sh 2>/dev/null || echo "Camera setup skipped (no camera?)"

# Step 2: Tailscale
echo "Step 2: Checking Tailscale..."
tailscale status 2>/dev/null | head -3 || echo "Tailscale not running"

# Step 3: Bridge
echo "Step 3: Launching Bridge..."
# Try SD card first, fall back to persistent dir
if [ -f /mnt/sdcard/drone_project/raxda_bridge/real_bridge_service.py ]; then
    cd /mnt/sdcard/drone_project/raxda_bridge/
    python3 real_bridge_service.py
elif [ -f /home/shreyash/drone_project_persistent/raxda_bridge/real_bridge_service.py ]; then
    cd /home/shreyash/drone_project_persistent/raxda_bridge/
    python3 real_bridge_service.py
else
    echo "ERROR: No bridge code found!"
    exit 1
fi
LAUNCH_EOF
chmod +x /home/$USER_NAME/launch_bridge.sh
chown $USER_NAME:$USER_NAME /home/$USER_NAME/launch_bridge.sh

# Systemd service
cat > /etc/systemd/system/drone-bridge.service << SERVICE_EOF
[Unit]
Description=Radxa Drone Bridge Service
After=network.target tailscaled.service

[Service]
Type=simple
User=$USER_NAME
ExecStartPre=/bin/sleep 10
ExecStart=/home/$USER_NAME/launch_bridge.sh
WorkingDirectory=/home/$USER_NAME
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
SERVICE_EOF

systemctl daemon-reload
systemctl enable drone-bridge
echo "  Drone bridge service created (auto-starts on boot)"

# ===== 12. SYMLINKS & CLEANUP =====
echo ""
echo "[12/12] Final setup..."
ln -sf /mnt/sdcard/drone_project /home/$USER_NAME/drone_project 2>/dev/null || true

# Create persistent dir on eMMC as fallback
mkdir -p /home/$USER_NAME/drone_project_persistent/raxda_bridge
chown -R $USER_NAME:$USER_NAME /home/$USER_NAME/drone_project_persistent

echo ""
echo "========================================"
echo " POST-FLASH SETUP COMPLETE!"
echo "========================================"
echo ""
echo " Hardware Ports:"
echo "   FC:        /dev/ttyS2 @ 57600 baud"
echo "   ESP32:     /dev/ttyS4 @ 115200 baud"
echo "   LiDAR:     /dev/ttyUSB0 @ 128000 baud"
echo "   Camera:    /dev/video0 (IMX219 1080p)"
echo ""
echo " Services:"
echo "   drone-bridge:     enabled (auto-start)"
echo "   thermal-watchdog: enabled (shutdown at 80C)"
echo "   tailscaled:       enabled"
echo ""
echo " NEXT STEPS:"
echo "   1. sudo tailscale up    (authenticate)"
echo "   2. sudo reboot          (apply overlays)"
echo "   3. After reboot, bridge auto-starts"
echo ""
echo " TO DEPLOY CODE REMOTELY (from laptop):"
echo "   ssh shreyash@100.94.63.59"
echo "   sudo mount -o remount,rw /mnt/sdcard"
echo "   scp files... /mnt/sdcard/drone_project/..."
echo "   sudo mount -o remount,ro /mnt/sdcard"
echo "   sudo systemctl restart drone-bridge"
echo ""
echo "========================================"
