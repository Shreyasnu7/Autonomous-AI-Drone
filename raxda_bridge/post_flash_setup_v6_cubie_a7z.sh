#!/bin/bash

# ---- Network credentials -------------------------------------------------
# Supplied by the operator, never committed. Either export these before
# running, or place them in /etc/drone-wifi.conf (chmod 600):
#     WIFI_SSID="my-hotspot"          WIFI_PASS="..."
#     WIFI_FALLBACK_SSID="my-router"  WIFI_FALLBACK_PASS="..."
[ -f /etc/drone-wifi.conf ] && . /etc/drone-wifi.conf
WIFI_SSID="${WIFI_SSID:-}"
WIFI_PASS="${WIFI_PASS:-}"
WIFI_FALLBACK_SSID="${WIFI_FALLBACK_SSID:-}"
WIFI_FALLBACK_PASS="${WIFI_FALLBACK_PASS:-}"
# --------------------------------------------------------------------------

# =====================================================
# RADXA CUBIE A7Z - POST-FLASH SETUP v6.0
# COMPLETE RESTORE — reflash + run this = back to 100%
# =====================================================
# Allwinner A733 | WiFi 6 (AIC8800) | BT 5.4
#
# Usage:
#   1. Flash Radxa Debian 11 CLI to SD card
#   2. Boot with monitor+keyboard OR UART
#   3. Login as root (or default user)
#   4. Connect WiFi: nmcli dev wifi connect "$WIFI_SSID" password "$WIFI_PASS"
#   5. Create user: adduser shreyash (password: 1)
#   6. SCP this script + raxda_bridge folder:
#      scp post_flash_setup_v6_cubie_a7z.sh shreyash@<ip>:~/
#      scp -r raxda_bridge/ shreyash@<ip>:~/
#      scp -r raxda/ shreyash@<ip>:~/
#   7. Run: chmod +x ~/post_flash_setup_v6_cubie_a7z.sh
#          sudo ~/post_flash_setup_v6_cubie_a7z.sh
# =====================================================

set -e

if [ "$EUID" -ne 0 ]; then
  echo "Please run as root (sudo)"
  exit 1
fi

echo "================================================"
echo " RADXA CUBIE A7Z - POST-FLASH SETUP v6.0"
echo " COMPLETE RESTORE SCRIPT"
echo "================================================"
echo ""

USER_NAME="shreyash"
USER_HOME="/home/$USER_NAME"
BRIDGE_DIR="$USER_HOME/raxda_bridge"

# ===== 0. USER + SSH (FIRST — never get locked out) =====
echo "[0] User + SSH..."
if ! id "$USER_NAME" &>/dev/null; then
    adduser --disabled-password --gecos "" $USER_NAME 2>/dev/null || true
    echo "$USER_NAME:1" | chpasswd
fi
usermod -aG sudo,dialout,video,bluetooth,i2c $USER_NAME 2>/dev/null || true

apt-get install -y openssh-server 2>/dev/null || true
systemctl enable ssh 2>/dev/null || systemctl enable sshd 2>/dev/null || true
systemctl start ssh 2>/dev/null || systemctl start sshd 2>/dev/null || true
sed -i 's/#PasswordAuthentication.*/PasswordAuthentication yes/' /etc/ssh/sshd_config 2>/dev/null || true
sed -i 's/PasswordAuthentication no/PasswordAuthentication yes/' /etc/ssh/sshd_config 2>/dev/null || true
systemctl restart ssh 2>/dev/null || systemctl restart sshd 2>/dev/null || true
echo "  ✅ SSH ON — ssh shreyash@$(hostname -I | awk '{print $1}')"

# Sudoers for drone operations
cat > /etc/sudoers.d/drone-ops << 'EOF'
shreyash ALL=(ALL) NOPASSWD: ALL
EOF
chmod 440 /etc/sudoers.d/drone-ops

# ===== 1. SYSTEM PACKAGES =====
echo "[1] System packages..."
apt-get update -y
apt-get install -y \
    python3 python3-pip python3-dev python3-opencv \
    git cmake build-essential v4l-utils i2c-tools \
    gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav \
    python3-gst-1.0 net-tools wireless-tools network-manager \
    bluez bluetooth curl htop rsync picocom \
    || echo "  Some packages may have failed"

# ===== 2. PYTHON PACKAGES =====
echo "[2] Python packages..."
pip3 install --break-system-packages \
    pymavlink pyserial websockets aiohttp numpy \
    smbus2 bleak opencv-python-headless lxml fastcrc \
    || echo "  Some pip packages may have failed"

# ===== 3. CLEANUP (free SD card space) =====
echo "[3] Disk cleanup..."
apt-get clean
apt-get autoremove -y 2>/dev/null || true
rm -rf /tmp/pip-* /root/.cache/pip /var/cache/apt/archives/*.deb 2>/dev/null || true
echo "  $(df -h / | tail -1 | awk '{print $4}') free"

# ===== 4. TAILSCALE =====
echo "[4] Tailscale..."
if ! command -v tailscale &>/dev/null; then
    curl -fsSL https://tailscale.com/install.sh | sh
fi
systemctl enable --now tailscaled
echo "  ✅ Installed (run 'sudo tailscale up' after setup)"

# ===== 5. WIFI AUTO-RECONNECT SERVICE =====
echo "[5] WiFi auto-reconnect service..."
cat > /usr/local/bin/wifi-reconnect.sh << 'WIFI_EOF'
#!/bin/bash
# WiFi auto-reconnect — ensures Cubie always has a network connection
# Priority: $WIFI_SSID (hotspot) > $WIFI_FALLBACK_SSID > any saved connection
# NEVER auto-connect to GoPro WiFi (GP*) — that kills internet

while true; do
    sleep 15

    # Check if we have internet
    if ping -c 1 -W 3 8.8.8.8 &>/dev/null; then
        continue  # Internet works, do nothing
    fi

    # Check if connected to GoPro WiFi (disaster — disconnect immediately)
    CURRENT=$(nmcli -t -f active,ssid dev wifi | grep '^yes' | cut -d: -f2)
    if [[ "$CURRENT" == GP* ]]; then
        echo "$(date): EMERGENCY — disconnecting from GoPro WiFi: $CURRENT"
        nmcli dev wifi disconnect 2>/dev/null || true
        sleep 2
    fi

    # Try phone hotspot first
    if nmcli dev wifi list 2>/dev/null | grep -q "$WIFI_SSID"; then
        echo "$(date): Connecting to $WIFI_SSID..."
        nmcli dev wifi connect "$WIFI_SSID" password "$WIFI_PASS" 2>/dev/null && continue
    fi

    # Try 4G dongle WiFi
    if nmcli dev wifi list 2>/dev/null | grep -q "$WIFI_FALLBACK_SSID"; then
        echo "$(date): Connecting to $WIFI_FALLBACK_SSID..."
        nmcli dev wifi connect "$WIFI_FALLBACK_SSID" password "$WIFI_FALLBACK_PASS" 2>/dev/null && continue
    fi

    # Try any saved connection
    nmcli dev wifi connect --ask 2>/dev/null || true
done
WIFI_EOF
chmod +x /usr/local/bin/wifi-reconnect.sh

cat > /etc/systemd/system/wifi-reconnect.service << 'EOF'
[Unit]
Description=WiFi Auto-Reconnect (drone safe)
After=NetworkManager.service
Wants=NetworkManager.service

[Service]
Type=simple
ExecStart=/usr/local/bin/wifi-reconnect.sh
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF
systemctl daemon-reload
systemctl enable wifi-reconnect
systemctl start wifi-reconnect
echo "  ✅ Auto-reconnect active (blocks GoPro WiFi)"

# ===== 6. WiFi CONNECTIONS (pre-save known networks) =====
echo "[6] Saving known WiFi networks..."
# Phone hotspot (priority 20 — preferred)
nmcli connection add type wifi con-name "$WIFI_SSID" ssid "$WIFI_SSID" \
    wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$WIFI_PASS" \
    connection.autoconnect yes connection.autoconnect-priority 20 \
    2>/dev/null || nmcli connection modify "$WIFI_SSID" connection.autoconnect yes connection.autoconnect-priority 20 2>/dev/null || true

# 4G dongle WiFi (priority 10 — fallback)
nmcli connection add type wifi con-name "$WIFI_FALLBACK_SSID" ssid "$WIFI_FALLBACK_SSID" \
    wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$WIFI_FALLBACK_PASS" \
    connection.autoconnect yes connection.autoconnect-priority 10 \
    2>/dev/null || nmcli connection modify "$WIFI_FALLBACK_SSID" connection.autoconnect yes connection.autoconnect-priority 10 2>/dev/null || true
echo "  ✅ $WIFI_SSID (priority 20) + $WIFI_FALLBACK_SSID (priority 10)"

# ===== 7. 4G DONGLE RNDIS USB DRIVER =====
echo "[7] 4G dongle RNDIS driver..."
# Load rndis_host module for HiMI UFI 4G dongle (USB ethernet)
modprobe rndis_host 2>/dev/null || true
if ! grep -q "rndis_host" /etc/modules 2>/dev/null; then
    echo "rndis_host" >> /etc/modules
fi
# Blacklist option driver (interferes with RNDIS)
echo "blacklist option" > /etc/modprobe.d/blacklist-option.conf 2>/dev/null || true
echo "  ✅ rndis_host loaded, option blacklisted"

# ===== 8. SERIAL DRIVERS + UART =====
echo "[8] Serial drivers + UART..."
modprobe cp210x 2>/dev/null || true
modprobe ch341 2>/dev/null || true
for mod in cp210x ch341; do
    grep -q "$mod" /etc/modules 2>/dev/null || echo "$mod" >> /etc/modules
done

# udev rules
cat > /etc/udev/rules.d/99-drone-serial.rules << 'EOF'
KERNEL=="ttyAS0", MODE="0666"
KERNEL=="ttyAS1", MODE="0666"
KERNEL=="ttyS0", MODE="0666"
KERNEL=="ttyUSB*", MODE="0666"
KERNEL=="video*", MODE="0666"
EOF
udevadm control --reload-rules 2>/dev/null || true

# Quiet kernel console
echo "kernel.printk = 0 0 0 0" > /etc/sysctl.d/99-quiet-console.conf
sysctl -p /etc/sysctl.d/99-quiet-console.conf 2>/dev/null || true
echo "  ✅ UART + USB serial ready"

# ===== 9. THERMAL GUARD =====
echo "[9] Thermal guard..."
cat > /usr/local/bin/thermal_guard.sh << 'EOF'
#!/bin/bash
while true; do
    TEMP=$(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null || echo "0")
    if [ "$TEMP" -gt "75000" ]; then
        for cpu in /sys/devices/system/cpu/cpufreq/policy*/scaling_max_freq; do
            echo "1000000" > "$cpu" 2>/dev/null; done
    elif [ "$TEMP" -gt "65000" ]; then
        for cpu in /sys/devices/system/cpu/cpufreq/policy*/scaling_max_freq; do
            echo "1600000" > "$cpu" 2>/dev/null; done
    else
        for cpu in /sys/devices/system/cpu/cpufreq/policy*/scaling_max_freq; do
            echo "2000000" > "$cpu" 2>/dev/null; done
    fi
    sleep 5
done
EOF
chmod +x /usr/local/bin/thermal_guard.sh

cat > /etc/systemd/system/thermal-guard.service << 'EOF'
[Unit]
Description=Thermal Guard
After=multi-user.target
[Service]
Type=simple
ExecStart=/usr/local/bin/thermal_guard.sh
Restart=always
[Install]
WantedBy=multi-user.target
EOF
systemctl daemon-reload
systemctl enable --now thermal-guard
echo "  ✅ Throttle 65°C, aggressive 75°C, NEVER shutdown"

# ===== 10. WATCHDOG + PANIC RECOVERY =====
echo "[10] Watchdog + panic recovery..."
sed -i 's/#RuntimeWatchdogSec=.*/RuntimeWatchdogSec=15/' /etc/systemd/system.conf 2>/dev/null || true
echo "softdog" > /etc/modules-load.d/watchdog.conf 2>/dev/null || true
cat > /etc/sysctl.d/99-drone-panic.conf << 'EOF'
kernel.panic = 10
kernel.sysrq = 1
EOF
sysctl --system 2>/dev/null || true
echo "  ✅ Auto-reboot on 15s freeze or kernel panic"

# ===== 11. FILESYSTEM HARDENING =====
echo "[11] Filesystem hardening..."
tune2fs -o journal_data $(findmnt -n -o SOURCE /) 2>/dev/null || true
# tmpfs for logs (prevent SD card wear)
grep -q "tmpfs /tmp" /etc/fstab || echo "tmpfs /tmp tmpfs defaults,noatime,nosuid,nodev,size=100m 0 0" >> /etc/fstab
grep -q "tmpfs /var/log" /etc/fstab || echo "tmpfs /var/log tmpfs defaults,noatime,nosuid,nodev,size=50m 0 0" >> /etc/fstab
# Safe shutdown alias
grep -q 'alias off=' $USER_HOME/.bashrc 2>/dev/null || echo 'alias off="sudo sync && sudo poweroff"' >> $USER_HOME/.bashrc
echo "  ✅ journal_data + tmpfs logs + 'off' alias"

# ===== 12. BRIDGE LAUNCHER =====
echo "[12] Bridge launcher..."
cat > $USER_HOME/launch_bridge.sh << 'LAUNCH_EOF'
#!/bin/bash
echo "=== CUBIE A7Z DRONE BRIDGE ==="

# Stop UART console so bridge can use ttyAS0 for FC
sudo systemctl stop serial-getty@ttyAS0.service 2>/dev/null
sudo chmod 666 /dev/ttyAS0 /dev/ttyAS1 2>/dev/null
trap "sudo systemctl start serial-getty@ttyAS0.service" EXIT

# Kill stale port locks
sudo fuser -k /dev/ttyAS0 8888/udp 8000/tcp 8080/tcp 2>/dev/null || true
sleep 0.5

# Find bridge (check both locations)
for DIR in "$HOME/raxda_bridge" "$HOME/drone_project/raxda_bridge"; do
    if [ -f "$DIR/real_bridge_service.py" ]; then
        cd "$DIR"
        exec python3 real_bridge_service.py
    fi
done

echo "ERROR: Bridge not found! SCP your code:"
echo "  scp -r raxda_bridge/ shreyash@$(hostname -I | awk '{print $1}'):~/"
exit 1
LAUNCH_EOF
chmod +x $USER_HOME/launch_bridge.sh
chown $USER_NAME:$USER_NAME $USER_HOME/launch_bridge.sh

# ===== 13. BRIDGE AUTO-START SERVICE =====
echo "[13] Bridge auto-start service..."
cat > /etc/systemd/system/drone-bridge.service << EOF
[Unit]
Description=Drone Bridge Service
After=network-online.target tailscaled.service
Wants=network-online.target

[Service]
Type=simple
User=$USER_NAME
ExecStartPre=/bin/sleep 10
ExecStart=$USER_HOME/launch_bridge.sh
WorkingDirectory=$USER_HOME
Restart=always
RestartSec=5
Environment=OPENCV_FFMPEG_READ_ATTEMPTS=1

[Install]
WantedBy=multi-user.target
EOF
systemctl daemon-reload
systemctl enable drone-bridge
echo "  ✅ Bridge auto-starts on boot"

# ===== 14. COPY BRIDGE CODE TO RIGHT LOCATION =====
echo "[14] Setting up bridge code..."
# If bridge files were SCP'd to ~/raxda_bridge, they're already in place
# Also check if they were SCP'd to ~/drone_project/raxda_bridge
if [ -d "$USER_HOME/raxda_bridge" ]; then
    echo "  ✅ Bridge code found at ~/raxda_bridge"
elif [ -d "$USER_HOME/drone_project/raxda_bridge" ]; then
    # Symlink for backward compatibility
    ln -sf "$USER_HOME/drone_project/raxda_bridge" "$USER_HOME/raxda_bridge" 2>/dev/null || true
    echo "  ✅ Bridge code found at ~/drone_project/raxda_bridge (symlinked)"
else
    mkdir -p "$USER_HOME/raxda_bridge"
    echo "  ⚠️ No bridge code found — SCP it after setup"
fi
chown -R $USER_NAME:$USER_NAME $USER_HOME/raxda_bridge 2>/dev/null || true

# Fix UART port names for Allwinner (ttyAS0 not ttyS0)
if [ -f "$USER_HOME/raxda_bridge/real_bridge_service.py" ]; then
    sed -i 's|"/dev/ttyS0"|"/dev/ttyAS0"|g' "$USER_HOME/raxda_bridge/real_bridge_service.py"
    sed -i 's|"/dev/ttyS2"|"/dev/ttyAS1"|g' "$USER_HOME/raxda_bridge/real_bridge_service.py"
    echo "  ✅ UART ports fixed (ttyAS0/ttyAS1)"
fi

# Also copy raxda/ module if present
if [ -d "$USER_HOME/raxda" ]; then
    echo "  ✅ raxda module found"
fi

# ===== 15. BLUETOOTH SETUP (GoPro pairing) =====
echo "[15] Bluetooth setup..."
systemctl enable bluetooth 2>/dev/null || true
systemctl start bluetooth 2>/dev/null || true
# Set pairable on boot
cat > /etc/bluetooth/main.conf.d/drone.conf 2>/dev/null << 'EOF' || true
[General]
Pairable = true
DiscoverableTimeout = 0
EOF
echo "  ✅ Bluetooth ready (pair GoPro via: bluetoothctl pair E1:9D:9A:BB:E5:5B)"

# ===== 16. ENVIRONMENT FILE =====
echo "[16] Environment config..."
cat > $USER_HOME/.drone_env << 'EOF'
# Drone environment variables
export CUBIE_TS_IP="100.64.0.30"
export LAPTOP_TS_IP="100.64.0.10"
export PHONE_TS_IP="100.64.0.20"
export GOPRO_MAC="E1:9D:9A:BB:E5:5B"
export FC_PORT="/dev/ttyAS0"
export FC_BAUD="57600"
export SERVER_URL="wss://drone-server-r0qe.onrender.com/ws/connect/RADXA_X"
export OPENCV_FFMPEG_READ_ATTEMPTS=1
EOF
chown $USER_NAME:$USER_NAME $USER_HOME/.drone_env
grep -q "source ~/.drone_env" $USER_HOME/.bashrc 2>/dev/null || \
    echo "source ~/.drone_env" >> $USER_HOME/.bashrc
echo "  ✅ Environment variables saved"

echo ""
echo "================================================"
echo " POST-FLASH SETUP v6.0 COMPLETE!"
echo "================================================"
echo ""
echo " ✅ SSH:           ON (shreyash / pw: 1)"
echo " ✅ WiFi:          $WIFI_SSID (auto-reconnect)"
echo " ✅ 4G Dongle:     RNDIS driver loaded"
echo " ✅ Tailscale:     Installed (run: sudo tailscale up)"
echo " ✅ Thermal:       65°C throttle, never shutdown"
echo " ✅ Watchdog:      15s freeze → reboot"
echo " ✅ Bridge:        Auto-start on boot"
echo " ✅ WiFi Guard:    Blocks GoPro WiFi hijack"
echo " ✅ Bluetooth:     Ready for GoPro pairing"
echo ""
echo " NEXT STEPS:"
echo "   1. sudo tailscale up    ← authenticate"
echo "   2. sudo reboot          ← apply all changes"
echo ""
echo " After reboot, bridge starts automatically."
echo " Manual start: ~/launch_bridge.sh"
echo " Manual stop:  sudo systemctl stop drone-bridge"
echo ""
echo " ⚠️  NEVER PULL POWER — type 'off' to shutdown"
echo "================================================"
