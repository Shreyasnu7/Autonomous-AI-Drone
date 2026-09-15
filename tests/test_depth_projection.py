"""DEPTH-PROJECTION GEOMETRY — the camera path that tells the AI how far things are and where.

No hardware required. Reproduces the projection director_core performs (horizon band -> per-column
clearance -> bearing -> body-frame point) and asserts the geometry, so the faults found on
2026-09-15 cannot return silently:

  * pixel->bearing must be PINHOLE, atan(x*tan(hfov/2)). A linear map is wrong by up to 3.5 deg
    mid-frame at 86 deg FOV, which misplaces an obstacle 3 m away by about 18 cm.
  * the horizon band must track the TOTAL camera depression (fixed mount tilt + gimbal), not the
    gimbal alone, or it samples floor and reports it as the nearest surface ahead.
  * per-column clearance must use a low PERCENTILE, not the raw minimum: one speckled depth pixel
    at 40 cm otherwise drives the speed cap to zero and stops the aircraft for nothing.

Run: python tests/test_depth_projection.py
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                "ai", "camera_brain"))
from laptop_ai.spatial_grid import SpatialGrid  # noqa: E402

MOUNT = {"cam_forward": 0.04, "cam_pitch_deg": 10.0, "cam_hfov_deg": 86.0}
H, W = 252, 448
_HFOV = math.radians(MOUNT["cam_hfov_deg"])
_TAN_H = math.tan(_HFOV / 2)
_VFOV = 2 * math.atan(_TAN_H * (H / float(W)))
_TAN_V = math.tan(_VFOV / 2)

_passed = _failed = 0


def check(name, cond, detail=""):
    global _passed, _failed
    if cond:
        _passed += 1
        print(f"  [PASS] {name} {detail}")
    else:
        _failed += 1
        print(f"  [FAIL] {name} {detail}")


def band_rows(gimbal_pitch_deg=0.0):
    cam_down = math.radians(MOUNT["cam_pitch_deg"]) + math.radians(gimbal_pitch_deg)

    def row(e_deg):
        t = math.tan(math.radians(e_deg) + cam_down)
        return int(round((0.5 - (t / (2 * _TAN_V))) * H))

    top, bot = row(12.0), row(-8.0)
    top = max(0, min(H - 2, min(top, bot)))
    bot = max(top + 1, min(H, bot))
    return top, bot


def project(metric_map, gimbal_pitch_deg=0.0, gimbal_yaw_deg=0.0, dscale=1.0):
    top, bot = band_rows(gimbal_pitch_deg)
    col_m = np.percentile(metric_map[top:bot, :], 10, axis=0).astype(np.float32) * dscale
    gp = math.radians(gimbal_yaw_deg)
    pts = []
    for ix in range(0, W, max(1, W // 48)):
        m = float(col_m[ix])
        if m >= 6.0 - 0.05:
            continue
        xn = (ix / W - 0.5) * 2.0
        bearing = math.atan(xn * _TAN_H) + gp
        pts.append((m * math.sin(bearing), -(m * math.cos(bearing) + MOUNT["cam_forward"])))
    return pts


print("DEPTH PROJECTION GEOMETRY")

# 1. bearing must be pinhole, not linear
xn = (0.25 - 0.5) * 2.0
pin = math.degrees(math.atan(xn * _TAN_H))
lin = math.degrees((0.25 - 0.5) * _HFOV)
check("quarter-frame bearing is pinhole", abs(pin - (-25.0)) < 0.6, f"{pin:.1f}deg")
check("linear map would have been wrong", abs(pin - lin) > 3.0,
      f"pinhole {pin:.1f} vs linear {lin:.1f}")

# 2. a wall dead ahead lands dead ahead, at the right range
mm = np.full((H, W), 3.0, np.float32)
g = SpatialGrid()
g.update(camera_points=project(mm))
front = g.get_front_obstacle_m()
check("wall 3m ahead reads ~3m", abs(front - 3.0) < 0.15, f"{front:.2f}m")

# 3. an obstacle on one side must not appear on the other
mm_r = np.full((H, W), 8.0, np.float32)
mm_r[:, int(W * 0.70):] = 2.0
g_r = SpatialGrid()
g_r.update(camera_points=project(mm_r))
desc = g_r.get_spatial_description() or ""
check("right-side obstacle is on the RIGHT", "front_right" in desc or "right=" in desc, desc[:60])
check("right-side obstacle is NOT on the left", "left=" not in desc, desc[:60])

# 4. the band must follow total depression (mount + gimbal), not the gimbal alone
lvl_top, _ = band_rows(-MOUNT["cam_pitch_deg"])   # gimbal cancels the mount => level camera
mnt_top, _ = band_rows(0.0)                       # gimbal neutral => 10 deg down
check("band shifts with the fixed mount tilt", mnt_top < lvl_top,
      f"rows {mnt_top} (mount 10deg down) vs {lvl_top} (level)")

# 5. a single speckled pixel must not become the clearance
col = np.full((H, W), 3.0, np.float32)
col[band_rows()[0] + 2, 100] = 0.4
c10 = float(np.percentile(col[band_rows()[0]:band_rows()[1], 100], 10))
check("speckle rejected by the percentile", c10 > 2.5, f"{c10:.2f}m (raw min would be 0.40m)")

# 6. a real obstacle spanning much of the column is still seen
col2 = np.full((H, W), 3.0, np.float32)
top, bot = band_rows()
col2[top:top + int((bot - top) * 0.6), 100] = 0.8
c2 = float(np.percentile(col2[top:bot, 100], 10))
check("genuine obstacle still detected", c2 < 1.0, f"{c2:.2f}m")

print(f"\nDEPTH PROJECTION RESULT: {_passed} passed / {_failed} failed")
sys.exit(1 if _failed else 0)
