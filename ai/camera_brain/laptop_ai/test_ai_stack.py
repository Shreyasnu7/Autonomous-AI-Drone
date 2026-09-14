"""Smoke-test the full laptop AI stack on a synthetic frame. Run from laptop_ai/."""
import os, time, traceback
import numpy as np
import cv2

def ok(name, msg=""):   print(f"[PASS] {name:22s} {msg}")
def bad(name, e):       print(f"[FAIL] {name:22s} {repr(e)[:120]}")

# Synthetic 480p frame with a couple shapes (so YOLO/depth have content)
frame = np.full((480, 640, 3), 60, np.uint8)
cv2.rectangle(frame, (200, 150), (340, 380), (180, 180, 180), -1)
cv2.circle(frame, (480, 240), 60, (120, 120, 220), -1)

print("="*60)
print("GPU:")
try:
    import torch
    print(f"  torch {torch.__version__} | CUDA {torch.cuda.is_available()} | {torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'CPU'}")
except Exception as e:
    print("  torch error:", e)
print("="*60)

# 1. YOLO
try:
    from ultralytics import YOLO
    m = YOLO("yolov8n.pt")
    r = m(frame, verbose=False)
    ok("YOLOv8n", f"{len(r[0].boxes)} boxes")
except Exception as e: bad("YOLOv8n", e)

# 2. Depth (Depth Anything V2 -> MiDaS fallback)
try:
    from ai_depth_estimator import AIDepthEstimator
    de = AIDepthEstimator(use_gpu=True)
    dm = de.estimate(frame)
    ok("Depth estimator", f"map {getattr(dm,'shape',None)} via {getattr(de,'model_type','?')}")
except Exception as e: bad("Depth estimator", e)

# 3. Spatial grid (2.5D cm fusion)
try:
    from spatial_grid import SpatialGrid
    sg = SpatialGrid()
    sg.update(lidar_points=[(1.2,-0.3),(0.0,-2.1)], tof_sensors={'t1':850,'t2':1300,'t3':3000,'t4':2000},
              depth_info={'subject_depth_m':1.7}, altitude=8.0)
    img = sg.render_map()
    ok("Spatial grid (cm)", sg.get_spatial_description()[:60])
except Exception as e: bad("Spatial grid (cm)", e)

# 4. Local ER Brain (Qwen2.5-VL-3B 4-bit) — load + one real inference
try:
    from local_er_brain import LocalERBrain
    er = LocalERBrain()
    t0 = time.time()
    if er.connect():
        load = time.time() - t0
        er.set_director_intent("Track the subject, keep it centered.")
        er.update_state(frame, {'tof':{'t1':85},'altitude':8.0,'spatial':'front=85cm'}, [{'class':'person','confidence':0.8}])
        dec, t1 = None, time.time()
        while time.time() - t1 < 40 and dec is None:
            dec = er.get_latest_decision(); time.sleep(0.2)
        if dec:
            infer = time.time() - t1
            ok("Qwen ER Brain", f"load {load:.0f}s, 1st inference {infer:.1f}s, keys={list(dec.keys())}")
        else:
            bad("Qwen ER Brain", RuntimeError("loaded but no decision in 40s"))
    else:
        bad("Qwen ER Brain", RuntimeError("connect() returned False"))
except Exception as e: bad("Qwen ER Brain", e); traceback.print_exc()

# 5. Gemini cloud director (key + SDK)
try:
    key = os.getenv("GEMINI_API_KEY")
    import google.generativeai as genai
    ok("Gemini SDK/key", f"key {'SET ('+key[:8]+'...)' if key else 'MISSING'}")
except Exception as e: bad("Gemini SDK/key", e)

try:
    import torch
    if torch.cuda.is_available():
        print(f"\nPeak VRAM used: {torch.cuda.max_memory_allocated()/1e9:.2f} GB / "
              f"{torch.cuda.get_device_properties(0).total_memory/1e9:.1f} GB")
except Exception: pass
print("Done.")
