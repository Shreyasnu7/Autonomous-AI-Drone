"""Inspect how the Qwen pilot REASONS — feed it fake scenes/sensors + a task and print its RAW output.
Run from ai/camera_brain:  python test_qwen_raw.py
"""
import cv2, numpy as np
from laptop_ai.local_er_brain import LocalERBrain

def fake_frame(dets, w=640, h=480, label="SIM SCENE"):
    img = np.full((h, w, 3), 60, np.uint8)
    cv2.rectangle(img, (0, h // 2), (w, h), (90, 90, 90), -1)   # floor
    for d in dets:
        b = d.get("box")
        if b:
            x1, y1, x2, y2 = b
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 200, 0), 2)
            cv2.putText(img, f"{d['class']} {d.get('distance_m','?')}m", (x1, max(12, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 1)
    cv2.putText(img, label, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    return img

def run(brain, name, intent, sensors, dets):
    frame = fake_frame(dets, label=name[:40])
    out, sec, stext = brain.debug_once([frame, frame], sensors, dets, intent=intent)
    print("\n" + "=" * 74)
    print(f"SCENARIO: {name}   (inference {sec*1000:.0f} ms)")
    print(f"TASK: {intent}")
    print("--- SENSOR TEXT QWEN SAW ---\n" + stext)
    print("--- QWEN RAW RESPONSE ---\n" + out)

CAPS = {"max_lean_deg": 35, "max_climb_ms": 2.5, "max_descent_ms": 1.5,
        "max_horiz_speed_ms": 4.0, "max_vert_accel_ms2": 2.5}

print("Loading Qwen2.5-VL-3B ... (~25s)")
b = LocalERBrain()
if not b.connect():
    print("MODEL LOAD FAILED"); raise SystemExit

run(b, "Find the doorway & head to it (chair blocks center, wall close on left)",
    "USER REQUEST: explore the room and go to the doorway. DIRECTOR PLAN: stay ~0.6m, find the exit, "
    "navigate to it avoiding the furniture, then hold at the doorway.",
    {"t1": 120, "t2": 300, "t3": 400, "t4": 75,
     "spatial": "FRONT=120cm LEFT=75cm RIGHT=300cm BACK=400cm", "spatial_closest_m": 75, "spatial_closest_dir": "LEFT",
     "altitude": 0.6, "depth_to_subject_m": 3.8, "battery": 78, "speed": 0.0,
     "flight_dynamics": {"airborne": True, "throttle_pct": 52, "cell_voltage_v": 3.85, "climb_rate_ms": 0.0, "roll_deg": 1, "pitch_deg": -1},
     "capabilities": CAPS},
    [{"class": "door", "confidence": 0.81, "distance_m": 3.8, "bearing": "front-left", "box": [60, 140, 150, 360]},
     {"class": "chair", "confidence": 0.90, "distance_m": 1.2, "bearing": "center", "box": [280, 230, 380, 400]}])

run(b, "Go to the red chair, stop ~1m in front",
    "USER REQUEST: go to the red chair and stop about 1 meter in front of it. DIRECTOR PLAN: approach "
    "head-on, decelerate smoothly, hold 1m away facing it.",
    {"t1": 250, "t2": 500, "t3": 500, "t4": 500,
     "spatial": "FRONT=250cm, sides clear", "spatial_closest_m": 250, "spatial_closest_dir": "FRONT",
     "altitude": 0.8, "depth_to_subject_m": 2.4, "battery": 70, "speed": 0.3,
     "flight_dynamics": {"airborne": True, "throttle_pct": 50, "cell_voltage_v": 3.8, "climb_rate_ms": 0, "roll_deg": 0, "pitch_deg": -2},
     "capabilities": CAPS},
    [{"class": "chair", "confidence": 0.88, "distance_m": 2.4, "bearing": "center", "box": [270, 200, 400, 420]}])

run(b, "Obstacle 0.5m dead ahead - find a clear path forward",
    "USER REQUEST: keep moving forward through the room. DIRECTOR PLAN: progress forward, avoid "
    "obstacles, find a clear route onward.",
    {"t1": 50, "t2": 90, "t3": 400, "t4": 280,
     "spatial": "FRONT=50cm RIGHT=90cm LEFT=280cm BACK=400cm", "spatial_closest_m": 50, "spatial_closest_dir": "FRONT",
     "altitude": 0.7, "depth_to_subject_m": 0.5, "battery": 60, "speed": 0.4,
     "flight_dynamics": {"airborne": True, "throttle_pct": 55, "cell_voltage_v": 3.7, "climb_rate_ms": 0, "roll_deg": 0, "pitch_deg": -3},
     "capabilities": CAPS},
    [{"class": "wall/obstacle", "confidence": 0.7, "distance_m": 0.5, "bearing": "center", "box": [200, 150, 460, 400]}])

print("\nDONE")
