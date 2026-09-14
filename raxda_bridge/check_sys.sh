#!/bin/bash
echo "🔍 CHECKING CAMERA HARDWARE..."

echo "--- 1. Overlays (uEnv.txt) ---"
cat /boot/uEnv.txt 2>/dev/null || echo "No uEnv.txt found"

echo "--- 2. Checking Kernel Modules ---"
lsmod | grep imx219
lsmod | grep rkisp

echo "--- 3. Installing I2C Tools ---"
sudo apt-get update >/dev/null 2>&1
sudo apt-get install -y i2c-tools >/dev/null 2>&1

echo "--- 4. Scanning I2C Bus 3 (Expected Camera Bus) ---"
sudo i2cdetect -y 3

echo "--- 5. Scanning I2C Bus 2 (Alternative) ---"
sudo i2cdetect -y 2

echo "🏁 DONE."
