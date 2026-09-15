"""DOME - spherical obstacle memory and active looking.

No hardware required. The flat spatial grid is a single horizontal slice, so it cannot say whether
something is above or below the flight path. The dome accumulates the LiDAR ring (tagged with the
attitude the aircraft already has) and the camera cone (tagged with where the gimbal is pointing)
into one azimuth x elevation picture.

These checks pin the properties that make it trustworthy rather than merely present:

  * frame conventions match the rest of the codebase (forward = -y in, azimuth 0 = forward out).
    The camera path got this wrong first time and put every obstacle behind the aircraft.
  * an unobserved direction returns None - UNKNOWN, never "clear".
  * sightings EXPIRE, and the memory rotates with the aircraft.
  * tilting the airframe genuinely converts a 2D scanner into dome coverage.
  * the scanner yields the sensors to any real task and stands down once it knows enough.

Run: python tests/test_dome.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                "ai", "camera_brain"))
from laptop_ai.spherical_memory import SphericalMemory   # noqa: E402
from laptop_ai.dome_scan import DomeScanner              # noqa: E402

_passed = _failed = 0


def check(name, cond, detail=""):
    global _passed, _failed
    if cond:
        _passed += 1
        print(f"  [PASS] {name} {detail}")
    else:
        _failed += 1
        print(f"  [FAIL] {name} {detail}")


def ring(rng_m=3.0, step=6):
    return [(rng_m * math.sin(math.radians(a)), -rng_m * math.cos(math.radians(a)))
            for a in range(0, 360, step)]


T = 1000.0
print("DOME MEMORY")

# 1. frame convention: the director stores camera points as (lat, -fwd)
d = SphericalMemory()
d.add_camera_cone([(0.0, -2.0)], gimbal_pitch_deg=0.0, now=T)
check("obstacle ahead reads ahead", abs((d.clearance(0, 0, 15) or 0) - 2.0) < 0.01,
      f"{d.clearance(0, 0, 15)}m")
check("obstacle ahead is NOT behind", d.clearance(180, 0, 15) is None)

d_r = SphericalMemory()
d_r.add_camera_cone([(2.0, 0.0)], now=T)
check("obstacle right reads right", abs((d_r.clearance(90, 0, 15) or 0) - 2.0) < 0.01)

# 2. unobserved is UNKNOWN, not clear
d2 = SphericalMemory()
d2.add_lidar_ring(ring(), roll_deg=0, pitch_deg=0, now=T)
check("level ring leaves overhead unknown", d2.clearance(0, 45, 15) is None, "None = unobserved")
check("level ring knows straight ahead", d2.clearance(0, 0, 10) is not None)

# 3. tilting the airframe turns a 2D scanner into a dome
cov_flat = d2.coverage()
d3 = SphericalMemory()
for p in (-40, -20, 0, 20, 40):
    d3.add_lidar_ring(ring(), pitch_deg=p, now=T)
check("airframe tilt multiplies coverage", d3.coverage() > cov_flat * 2.5,
      f"{cov_flat*100:.0f}% flat -> {d3.coverage()*100:.0f}% swept")

# 4. sightings expire
d4 = SphericalMemory()
d4.add_lidar_ring(ring(), now=T)
d4.update_ego(now=T + 1.0)
check("fresh sighting survives", d4.clearance(0, 0, 10) is not None)
d4.update_ego(now=T + SphericalMemory.MEM_SECS + 1.0)
check("stale sighting expires", d4.clearance(0, 0, 10) is None, "not an obstacle forever")

# 5. the memory rotates with the aircraft
d5 = SphericalMemory()
d5.add_camera_cone([(0.0, -2.0)], now=T)                      # 2 m dead ahead
d5.update_ego(dyaw_rad=math.radians(90.0), now=T + 0.1)       # aircraft yaws right 90
check("yaw moves the obstacle to the left", d5.clearance(-90, 0, 20) is not None,
      "was ahead, now on the left")
check("nothing is left ahead after the turn", d5.clearance(0, 0, 10) is None)

print("\nACTIVE LOOKING")
s = DomeScanner()
empty = SphericalMemory()
aim = s.next_gimbal(empty, busy=False, travel_az_deg=30.0, now=T + 1)
check("scans an empty dome", aim is not None, f"aim {aim}")
check("yields to a real task", s.next_gimbal(empty, busy=True, now=T + 2) is None)
check("no airframe tilt while grounded", s.tilt_bias(empty, airborne=False, now=T) == 0.0)
check("no airframe tilt while busy", s.tilt_bias(empty, airborne=True, busy=True, now=T) == 0.0)
bias = s.tilt_bias(empty, airborne=True, busy=False, now=T + 1)
check("tilt bias stays small", abs(bias) <= DomeScanner.TILT_BIAS_DEG + 1e-6, f"{bias:+.1f}deg")

full = SphericalMemory()
for p in (-45, -30, -15, 0, 15, 30, 45):
    full.add_lidar_ring(ring(), pitch_deg=p, now=T)
s2 = DomeScanner()
s2._active = True
check("stands down once it knows enough",
      s2.next_gimbal(full, busy=False, now=T + 5) is None,
      f"coverage {full.coverage()*100:.0f}%")

print(f"\nDOME RESULT: {_passed} passed / {_failed} failed")
sys.exit(1 if _failed else 0)
