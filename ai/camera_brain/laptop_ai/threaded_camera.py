
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
            
        self.src = src                      # kept so the stream can be reopened after a drop
        self.grabbed, self.frame = self.stream.read()
        self.frame_t = time.time() if self.frame is not None else 0.0
        self.stopped = False
        self.lock = threading.Lock()

    def start(self):
        if self.working:
            threading.Thread(target=self.update, args=(), daemon=True).start()
        return self

    # A frame older than this is not shown to the AI. Flying on a frozen image is the worst
    # case here: every downstream layer (detector, depth, obstacle points) would keep
    # describing a scene the aircraft has already left, with full confidence.
    MAX_FRAME_AGE_S = 1.0
    RECONNECT_AFTER = 30          # consecutive failed reads before reopening the stream

    def update(self):
        fails = 0
        while not self.stopped:
            try:
                grabbed, frame = self.stream.read()
            except Exception as e:
                # An exception used to propagate out of this thread and kill it silently,
                # leaving the last frame served forever.
                grabbed, frame = False, None
                if fails == 0:
                    print(f"⚠️ Camera read error: {e}")

            if grabbed and frame is not None:
                fails = 0
                with self.lock:
                    self.grabbed = True
                    self.frame = frame
                    self.frame_t = time.time()
            else:
                fails += 1
                if fails == 1:
                    print("⚠️ Camera stream returned no frame - retrying")
                # The stream does not recover on its own; reopen it rather than going blind
                # for the rest of the flight.
                if fails % self.RECONNECT_AFTER == 0 and self.src is not None:
                    try:
                        self.stream.release()
                    except Exception:
                        pass
                    try:
                        self.stream = (cv2.VideoCapture(self.src, cv2.CAP_DSHOW)
                                       if isinstance(self.src, int) else cv2.VideoCapture(self.src))
                        try:
                            self.stream.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                        except Exception:
                            pass
                        print(f"🔁 Camera reconnect attempt ({fails} failed reads)")
                    except Exception as e:
                        print(f"⚠️ Camera reconnect failed: {e}")
                time.sleep(0.05)
            time.sleep(0.001) # Yield a tiny bit

    def read(self):
        """Freshest frame, or None if the stream has stalled.

        Returning a stale frame is worse than returning nothing: the caller cannot tell the
        difference between a still scene and a dead feed, and would keep acting on it.
        """
        with self.lock:
            if self.frame is None:
                return None
            if time.time() - getattr(self, 'frame_t', 0.0) > self.MAX_FRAME_AGE_S:
                return None
            return self.frame.copy()

    def stop(self):
        self.stopped = True
        if self.stream: 
            self.stream.release()
