
import cv2
import time

print("📸 V4L2 FORMAT DIAGNOSTIC")
print("========================")

def test_pipeline(name, pipeline):
    print(f"\n👉 Testing {name}...")
    print(f"   Pipeline: {pipeline}")
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print(f"❌ Failed to open {name}")
        return
    
    # Try to read
    for i in range(10): 
        ret = cap.grab() # Fast grab
        if not ret: break
    
    ret, frame = cap.read()
    if ret:
        fname = f"img_{name}.jpg"
        cv2.imwrite(fname, frame)
        print(f"✅ Saved {fname} ({frame.shape})")
    else:
        print(f"❌ Failed to read frame from {name}")
    
    cap.release()

# Common settings
# Using io-mode=2 (MMAP) to avoid DMABUF hang/slowdown
base = "v4l2src device=/dev/video0 io-mode=2 ! video/x-raw,width=640,height=480,framerate=30/1"

# 1. NV12 (Standard)
test_pipeline("nv12", f"{base},format=NV12 ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true sync=false")

# 2. UYVY (Packed)
test_pipeline("uyvy", f"{base},format=UYVY ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true sync=false")

# 3. YU12 (Planar)
test_pipeline("yu12", f"{base},format=I420 ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true sync=false")

print("\n🏁 DONE.")
