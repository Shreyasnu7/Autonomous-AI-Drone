
import cv2
import time

print("📸 FAST DIAGNOSTIC (Post-Update)")
print("==============================")

def save(name, frame):
    if frame is None:
        print(f"❌ {name}: No Frame")
        return
    filename = f"{name}.jpg"
    cv2.imwrite(filename, frame)
    print(f"✅ Saved {filename}")

# 1. Standard OpenCV (V4L2) - Was likely 'Dark' before
print("\n👉 1. Testing Standard OpenCV (v4l2src)...")
cap = cv2.VideoCapture(0)
# Force high gain/exposure if possible? No, just capture default.
if cap.isOpened():
    for i in range(10): cap.read() # Warmup
    ret, frame = cap.read()
    save("img_standard", frame)
    cap.release()
else:
    print("❌ Failed to Open V4L2")

# 2. Libcamera (Modern) - Was likely 'Purple' before?
print("\n👉 2. Testing Libcamera (libcamerasrc)...")
pipeline = "libcamerasrc ! video/x-raw,width=640,height=480,format=NV12 ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true sync=false"
cap_lib = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
if cap_lib.isOpened():
    for i in range(20): cap_lib.read()
    ret, frame = cap_lib.read()
    save("img_libcamera", frame)
    cap_lib.release()
else:
    print("❌ Failed to Open Libcamera")

print("\n🏁 DONE.")
