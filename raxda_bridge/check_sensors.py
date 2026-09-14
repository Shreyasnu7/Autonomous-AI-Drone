#!/usr/bin/env python3
import serial
import time
import sys

# ESP32 Port (ttyS4 on Radxa Zero 3)
ESP_PORT = "/dev/ttyS4"
ESP_BAUD = 115200  # Usually 115200, but could be 9600 or 57600

# Lidar Port (Using simple serial check)
LIDAR_PORT = "/dev/ttyUSB0"

def check_esp32():
    print(f"📡 CHECKING ESP32 on {ESP_PORT} @ {ESP_BAUD}...")
    try:
        ser = serial.Serial(ESP_PORT, ESP_BAUD, timeout=1)
        print("✅ Port Opened! Listening for 5 seconds...")
        start = time.time()
        while time.time() - start < 5:
            if ser.in_waiting:
                raw = ser.read(ser.in_waiting)
                try:
                    text = raw.decode('utf-8', errors='ignore').strip()
                    if text:
                        print(f"👉 DATA: {text}")
                    else:
                        print(f"👉 RAW: {raw}")
                except:
                    print(f"👉 HEX: {raw.hex()}")
            time.sleep(0.1)
        ser.close()
    except Exception as e:
        print(f"❌ Error opening {ESP_PORT}: {e}")
        print("   (Ensure you added 'uart4' overlay via setup script!)")

def check_lidar():
    print(f"\n🔭 CHECKING LIDAR on {LIDAR_PORT}...")
    try:
        ser = serial.Serial(LIDAR_PORT, 128000, timeout=1) # X2/X4 usually 128000
        ser.write(b'\xAA\x55\xF0\xEB\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00') # Stop
        time.sleep(0.1)
        ser.reset_input_buffer()
        ser.write(b'\xA5\x60') # Scan Command (X2)
        time.sleep(0.5)
        raw = ser.read(100)
        if len(raw) > 10:
            print(f"✅ LIDAR Responded! ({len(raw)} bytes)")
            print(f"   Hex: {raw[:20].hex()}...")
        else:
            print("⚠️ Lidar Silent (Might be different model or strict SDK needed).")
        ser.close()
    except Exception as e:
        print(f"❌ Lidar Error: {e}")

if __name__ == "__main__":
    check_esp32()
    check_lidar()
