
import cv2
import threading
import time

class CameraStream:
    def __init__(self, src=None, width=640, height=480, fps=30):
        # Fix for Windows UDP Stream: Don't force DSHOW for network strings
        if src is None:
             raise Exception("CameraStream: 'src' must be specified! (Use src=0 for webcam if intended, do not rely on default)")

        if isinstance(src, int):
             self.stream = cv2.VideoCapture(src, cv2.CAP_DSHOW)
        else:
             self.stream = cv2.VideoCapture(src) # Let OpenCV auto-detect (FFMPEG usually)

        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.stream.set(cv2.CAP_PROP_FPS, fps)
        try:
            self.stream.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # keep only the freshest frame — kills the RTSP lag backlog
        except Exception:
            pass
        
        if not self.stream.isOpened():
            print("⚠️ Camera Stream Failed to Open")
            self.working = False
        else:
            self.working = True
            
        self.grabbed, self.frame = self.stream.read()
        self.stopped = False
        self.lock = threading.Lock()

    def start(self):
        if self.working:
            threading.Thread(target=self.update, args=(), daemon=True).start()
        return self

    def update(self):
        while not self.stopped:
            grabbed, frame = self.stream.read()
            with self.lock:
                self.grabbed = grabbed
                self.frame = frame
            time.sleep(0.001) # Yield a tiny bit

    def read(self):
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.stopped = True
        if self.stream: 
            self.stream.release()
