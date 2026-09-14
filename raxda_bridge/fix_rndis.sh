#!/bin/bash
set -e
echo "=== RNDIS FIX SCRIPT ==="

cd /tmp
rm -f rndis_host.c rndis_host.h rndis_host.ko rndis_host.o rndis_host.mod.o rndis_host.mod.c Module.symvers modules.order

echo "[1] Downloading kernel source..."
wget -q https://raw.githubusercontent.com/torvalds/linux/v5.15/drivers/net/usb/rndis_host.c
wget -q https://raw.githubusercontent.com/torvalds/linux/v5.15/drivers/net/usb/rndis_host.h

echo "[2] Adding device ID..."
sed -i '/static const struct usb_device_id.*products/a\\t/* HiMI UFI 4G dongle */\n\t{USB_DEVICE(0x05c6, 0x9024), .driver_info = (unsigned long)\&rndis_info},' rndis_host.c

echo "[3] Patching force-bind..."
# Find the line with usbnet_generic_cdc_bind and replace the error handling
python3 -c "
lines = open('rndis_host.c').readlines()
out = []
i = 0
while i < len(lines):
    if 'retval = usbnet_generic_cdc_bind(dev, intf);' in lines[i]:
        out.append(lines[i])
        # Skip the next two lines (if retval < 0 / goto fail)
        i += 1
        while i < len(lines) and ('if (retval' in lines[i] or 'goto fail' in lines[i]):
            i += 1
        # Insert force-bind code
        out.append('\tif (retval < 0) {\n')
        out.append('\t\t/* Force bind for bad CDC descriptors (HiMI UFI 4G) */\n')
        out.append('\t\tdev->in = usb_rcvbulkpipe(dev->udev, 0x81);\n')
        out.append('\t\tdev->out = usb_sndbulkpipe(dev->udev, 0x02);\n')
        out.append('\t\tdev->status = NULL;\n')
        out.append('\t\tdev_info(&dev->udev->dev, \"RNDIS force-bind (bad CDC)\\\n\");\n')
        out.append('\t\tretval = 0;\n')
        out.append('\t}\n')
    else:
        out.append(lines[i])
        i += 1
open('rndis_host.c', 'w').writelines(out)
print('Patched OK')
"

echo "[4] Creating Makefile..."
cat > Makefile << 'EOF'
obj-m += rndis_host.o
KDIR := /lib/modules/$(shell uname -r)/build
all:
	make -C $(KDIR) M=$(PWD) modules
clean:
	make -C $(KDIR) M=$(PWD) clean
EOF

echo "[5] Removing old modules..."
sudo rmmod rndis_host_patched 2>/dev/null || true
sudo rmmod rndis_host 2>/dev/null || true
sudo rm -f /lib/modules/$(uname -r)/kernel/drivers/net/usb/rndis_patch.ko 2>/dev/null

echo "[6] Compiling..."
make clean 2>/dev/null || true
make
if [ ! -f rndis_host.ko ]; then
    echo "COMPILE FAILED"
    exit 1
fi

echo "[7] Installing..."
sudo cp rndis_host.ko /lib/modules/$(uname -r)/kernel/drivers/net/usb/
sudo depmod -a

echo "[8] Loading..."
sudo modprobe rndis_host
sleep 3

echo "[9] Setting up network..."
if ip link show usb0 &>/dev/null; then
    echo "usb0 exists!"
    sudo ip addr add 192.168.100.100/24 dev usb0 2>/dev/null || true
    sudo ip route add 192.168.100.0/24 dev usb0 2>/dev/null || true
    sudo ip route add default via 192.168.100.1 dev usb0 metric 50 2>/dev/null || true
    sudo adb shell service call connectivity 33 i32 1 s16 rndis 2>/dev/null || true
    echo "nameserver 8.8.8.8" | sudo tee /etc/resolv.conf > /dev/null

    echo "[10] Testing TCP through usb0..."
    sleep 2
    RESULT=$(curl --interface usb0 --connect-timeout 5 -s http://ifconfig.me 2>/dev/null)
    if [ -n "$RESULT" ]; then
        echo "=== TCP WORKS! Public IP: $RESULT ==="
        echo "=== RNDIS FIX COMPLETE ==="
    else
        echo "=== TCP through usb0 FAILED ==="
        echo "Ping test:"
        ping -I usb0 -c 2 192.168.100.1
    fi
else
    echo "usb0 MISSING - check dmesg:"
    sudo dmesg | grep -E "rndis|usb0" | tail -5
fi
