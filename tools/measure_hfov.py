"""Measure the camera's ACTUAL horizontal field of view.

Every distance, object size and depth scale the AI reports is derived from
f = (width/2) / tan(HFOV/2). The code ships an assumption of 86 degrees (GoPro HERO12
"Linear"), but the camera's mode and any digital zoom change it -- Wide is about 120 degrees
and an 0.5x ultrawide setting nearer 148. Getting this wrong scales every measurement by a
constant factor: assuming 86 while the camera is in Wide overstates sizes and depths by 86%.

Measure it once, per camera mode you fly in, and set CAM_HFOV_DEG.

    1. Put a flat object of KNOWN width square-on to the lens, centred, at a KNOWN distance.
       A door, a table edge or a tape measure held horizontally all work. Bigger is better.
    2. Capture a frame in the SAME camera mode and resolution you fly with.
    3. Read off the object's left and right edge pixel columns (any image viewer shows them).
    4. Run this with those numbers.

    python tools/measure_hfov.py --width-m 0.90 --distance-m 3.00 \
                                --x1 412 --x2 868 --frame-width 1280

It prints the measured HFOV and the error you are currently flying with.
"""
import argparse
import math


def main():
    ap = argparse.ArgumentParser(description="Measure camera HFOV from a known object.")
    ap.add_argument("--width-m", type=float, required=True, help="true width of the object, metres")
    ap.add_argument("--distance-m", type=float, required=True, help="lens-to-object distance, metres")
    ap.add_argument("--x1", type=float, required=True, help="left edge pixel column")
    ap.add_argument("--x2", type=float, required=True, help="right edge pixel column")
    ap.add_argument("--frame-width", type=float, required=True, help="frame width in pixels")
    ap.add_argument("--assumed", type=float, default=86.0, help="HFOV the code currently assumes")
    a = ap.parse_args()

    px = abs(a.x2 - a.x1)
    if px < 5:
        raise SystemExit("object spans too few pixels to measure reliably")
    if a.width_m <= 0 or a.distance_m <= 0:
        raise SystemExit("width and distance must be positive")

    f = px * a.distance_m / a.width_m                     # pinhole: px = f * W / D
    hfov = math.degrees(2 * math.atan(a.frame_width / (2 * f)))
    f_assumed = (a.frame_width / 2) / math.tan(math.radians(a.assumed) / 2)
    err = (f_assumed / f - 1.0) * 100.0

    print(f"object spans      : {px:.0f} px")
    print(f"focal length      : {f:.1f} px")
    print(f"MEASURED HFOV     : {hfov:.1f} deg")
    print(f"code assumes      : {a.assumed:.1f} deg  (f = {f_assumed:.1f} px)")
    print(f"current error     : {err:+.1f}% on every size and depth")
    print()
    if abs(err) < 3:
        print("Within 3% -- no change needed.")
    else:
        print(f"Set it before flying:   CAM_HFOV_DEG={hfov:.1f}")


if __name__ == "__main__":
    main()
