"""Export the production YOLO-World detector to a TensorRT FP16 engine and benchmark it.

Open-vocabulary models require their class vocabulary to be BAKED IN at export time. This script
sets the production navigation vocabulary before export, so the resulting engine detects exactly
the classes the director expects, and verifies the engine's class names match afterwards.

Measured on an RTX 5070 Ti: 24.0 ms -> 11.5 ms (2.10x) at 640 px.

Usage
-----
    python scripts/build_trt_world.py

Then copy the engine next to the director's launch directory:
    cp yolov8s-worldv2.engine ai/camera_brain/

The director loads it automatically when present (YOLO_TRT=1, the default) and falls back to the
PyTorch weights otherwise.

IMPORTANT
---------
  * Engines are GPU-architecture specific. Rebuild after any GPU change.
  * Changing WORLD_CLASSES requires re-exporting — the vocabulary is frozen in the engine.
  * Keep this list identical to ThreadedYOLO.WORLD_CLASSES in
    ai/camera_brain/laptop_ai/director_core.py
"""
import os
import time

import numpy as np
from ultralytics import YOLO, YOLOWorld

# Must match ThreadedYOLO.WORLD_CLASSES in director_core.py
WORLD_CLASSES = [
    "chair", "table", "sofa", "couch", "doorway", "door", "wall", "person",
    "potted plant", "refrigerator", "tv", "window", "cabinet", "stairs", "box",
    "shelf", "lamp", "obstacle", "furniture", "pillar", "railing",
]

WEIGHTS = os.getenv("YOLO_WORLD_MODEL", "yolov8s-worldv2.pt")
ENGINE = WEIGHTS.replace(".pt", ".engine")
IMGSZ = 640


def bench(fn, n=30, warmup=8):
    for _ in range(warmup):
        fn()
    t0 = time.time()
    for _ in range(n):
        fn()
    return (time.time() - t0) / n * 1000.0


def main():
    dummy = np.random.randint(0, 255, (480, 640, 3), np.uint8)

    model = YOLOWorld(WEIGHTS)
    model.set_classes(WORLD_CLASSES)
    pt_ms = bench(lambda: model.predict(dummy, verbose=False, imgsz=IMGSZ, device=0, conf=0.12))
    print(f"YOLO-World PyTorch : {pt_ms:6.1f} ms", flush=True)

    if not os.path.exists(ENGINE):
        print("exporting to TensorRT FP16 (vocabulary baked in, one-time, ~2-4 min) ...", flush=True)
        model.export(format="engine", half=True, imgsz=IMGSZ, device=0, workspace=4)

    engine = YOLO(ENGINE)
    trt_ms = bench(lambda: engine.predict(dummy, verbose=False, imgsz=IMGSZ, device=0, conf=0.12))
    print(f"YOLO-World TensorRT: {trt_ms:6.1f} ms   ({pt_ms / trt_ms:.2f}x faster)", flush=True)

    names = list(engine.names.values())
    print(f"engine classes: {names[:6]} ... ({len(names)} total)")
    assert names == WORLD_CLASSES, (
        "ENGINE VOCABULARY MISMATCH — the exported engine does not detect the production classes. "
        "Delete the engine and re-export."
    )
    print("CLASS NAMES MATCH THE PRODUCTION VOCABULARY — OK")


if __name__ == "__main__":
    main()
