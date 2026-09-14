import ydlidar
import time
import sys

print("-- YDLIDAR TEST --")
try:
    ports = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0"]
    bauds = [115200, 128000, 230400]

    for port in ports:
        for baud in bauds:
            print(f"Trying {port} @ {baud}...")
            laser = ydlidar.CYdLidar()
            laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
            laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, baud)
            laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
            laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
            laser.setlidaropt(ydlidar.LidarPropScanFrequency, 5.0)
            laser.setlidaropt(ydlidar.LidarPropSampleRate, 3)
            laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)

            if laser.initialize():
                print(f"✅ SUCCESS! Found Lidar on {port} @ {baud}")
                laser.turnOn()
                scan = ydlidar.LaserScan()
                for _ in range(5):
                    if laser.doProcessSimple(scan):
                        print(f"   Scan Data Points: {scan.points.size()}")
                        break
                    time.sleep(0.1)
                laser.turnOff()
                laser.disconnecting()
                sys.exit(0) # Found it!
            else:
                print(f"   Failed.")
                laser.disconnecting()
                
    print("❌ ALL FAILED.")
except Exception as e:
    print(f"❌ CRITICAL ERROR: {e}")
