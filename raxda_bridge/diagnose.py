
import cv2
import time
import os

print("📸 CAMERA DIAGNOSTIC TOOL")
print("========================")

def save_image(name, frame):
    if frame is None:
        print(f"❌ {name}: No Frame")
        return
    filename = f"{name}.jpg"
    try:
        cv2.imwrite(filename, frame)
        print(f"✅ {name}: Saved {filename} ({frame.shape})")
    except Exception as e:
        print(f"❌ {name}: Save Failed ({e})")

def test_pipeline(name, pipeline_str):
    print(f"\n👉 Testing: {name}")
    print(f"   Pipeline: {pipeline_str}")
    cap = cv2.VideoCapture(pipeline_str, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("   ❌ Failed to Open")
        return
    
    # Warmup
    for i in range(20):
        cap.read()
    
    # Capture
    ret, frame = cap.read()
    cap.release()
    
    if ret:
        save_image(name, frame)
    else:
        print("   ❌ Capture Failed (Empty Frame)")

# 1. Standard OpenCV
print("\n👉 Testing: Standard OpenCV (v4l2)")
cap = cv2.VideoCapture(0)
if cap.isOpened():
    for i in range(20): cap.read()
    ret, frame = cap.read()
    if ret: save_image("test_standard", frame)
    else: print("❌ Capture Failed")
    cap.release()
else:
    print("❌ Failed to Open /dev/video0")

# 2. GStreamer Auto
test_pipeline("test_gst_auto", "v4l2src device=/dev/video0 ! videoconvert ! appsink")

# 3. GStreamer NV12
test_pipeline("test_gst_nv12", "v4l2src device=/dev/video0 ! video/x-raw,format=NV12,width=1920,height=1080 ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true sync=false")

# 4. GStreamer DMABuf
test_pipeline("test_gst_dmabuf", "v4l2src device=/dev/video0 io-mode=4 ! video/x-raw,format=NV12,width=1920,height=1080 ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true sync=false")

# 5. libcamerasrc
test_pipeline("test_libcamera", "libcamerasrc ! video/x-raw,width=640,height=480 ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true sync=false")

print("\n🏁 DONE. Check generated .jpg files.")
