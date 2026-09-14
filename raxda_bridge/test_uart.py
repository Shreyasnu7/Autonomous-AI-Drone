
import serial
import time
import sys

SERIAL_PORT = "/dev/ttyS4"
BAUD_RATE = 115200

print(f"🔌 Opening {SERIAL_PORT} at {BAUD_RATE}...")

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print("✅ Port Opened! Listening for ESP32 Data...")
except Exception as e:
    print(f"❌ Failed to open port: {e}")
    sys.exit(1)

while True:
    try:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"📥 RAW: {line}")
        time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n👋 Exiting...")
        break
    except Exception as e:
        print(f"⚠️ Error: {e}")
