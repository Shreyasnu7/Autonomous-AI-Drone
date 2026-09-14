
import serial
import time
import sys
import glob

# Try all likely ports
PORTS = glob.glob('/dev/ttyS*') + glob.glob('/dev/ttyAML*')
BAUD_RATE = 115200

print(f"🔍 Scanning Ports: {PORTS}")

for p in PORTS:
    print(f"\n👉 Checking {p} ...")
    try:
        ser = serial.Serial(p, BAUD_RATE, timeout=0.5)
        # Clear buffer
        ser.reset_input_buffer()
        
        start = time.time()
        found_data = False
        
        while time.time() - start < 2.0: # Listen for 2 seconds
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line and line.startswith('{') and 't1' in line:
                    print(f"✅ FOUND DATA on {p}!")
                    print(f"📥 {line}")
                    found_data = True
                    break
        
        ser.close()
        
        if found_data:
            print(f"🎉 SUCCESS! The correct port is: {p}")
            sys.exit(0)
            
    except Exception as e:
        print(f"❌ Failed to open {p}: {e}")

print("\n❌ No ESP32 data found on any port.")
print("Check wiring (TX/RX swapped?) or Power.")
