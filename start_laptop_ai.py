import asyncio
import cv2
import json
import time
import aiohttp
from ultralytics import YOLO

# --- CONFIGURATION ---
# Use wss:// for secure websocket
SERVER_URL = "wss://web-production-fdccc.up.railway.app/ws/connect/laptop_vision"
MODEL_PATH = "yolov8n.pt"
# ---------------------

model = None

async def run_vision_loop(websocket, director=None, master_brain=None, tone_engine=None):
    """
    Main loop: Capture -> Master Brain (Decide) -> Tone -> Display -> Vision
    """
    cap = cv2.VideoCapture(0) 
    print("👀 Vision System Active - Looking for objects...")
    
    last_send = 0
    drone_pos = [0, 0, 0]
    
    # Scene stats for adaptive curve
    scene_data = {
        "total_clipping": 0.05,
        "scene_key": 0.5,
        "r_clip": 0.0, "g_clip": 0.0, "b_clip": 0.0
    }
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        display_frame = frame.copy()

        # 1. AI MASTER BRAIN (The "God Class")
        # Decides: Exposure, Focus, Stabilizer, Color Grade, Scene Type
        brain_plan = {}
        if master_brain:
            try:
                # We mock "fusion_state" (telemetry) and "vision_context" (detections) for now
                # In next step we pass real detections
                brain_plan = master_brain.decide(
                    user_text="", # could come from server
                    fusion_state={"gyro": {"gx":0, "gy":0, "gz":0}},
                    frame=frame,
                    vision_context={"detections": []} # Will populate below if re-ordered
                )
                
                # Apply Brain's Stabilization (if it returns a transform)
                # For now we just show we are "Thinking"
                mode = brain_plan.get("scene", {}).get("action", 0) > 0.5 and "ACTION" or "CINEMATIC"
                cv2.putText(display_frame, f"BRAIN: {mode} MODE", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)
                
                exp = brain_plan.get("exposure", {})
                if exp:
                    cv2.putText(display_frame, f"ISO:{int(exp.get('iso',100))} SHUTTER:{exp.get('shutter',0):.4f}", (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,200), 1)

            except Exception as e:
                print(f"Brain Error: {e}")

        # 2. Apply Global Tone Curve (ACES)
        if tone_engine:
             try:
                mean_lum = frame.mean() / 255.0
                scene_data["scene_key"] = mean_lum
                display_frame = tone_engine.apply_adaptive(display_frame, scene_data)
                cv2.putText(display_frame, "ACES GRADING", (10, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
             except Exception as e:
                pass
        
        # 3. Vision Inference
        results = model(frame, stream=True, verbose=False)
        detections = []
        primary_target = None
        
        for r in results:
             boxes = r.boxes
             for box in boxes:
                cls_id = int(box.cls[0])
                cls_name = model.names[cls_id]
                conf = float(box.conf[0])
                if conf > 0.5:
                    if cls_name in ["person", "car", "dog"]:
                         primary_target = box.xywh[0].tolist()

                    detections.append({
                        "label": cls_name, # Brain expects 'label'
                        "class": cls_name,
                        "confidence": round(conf, 2),
                        "bbox": box.xyxy[0].tolist(),
                        "box": box.xyxy[0].tolist(), # Brain expects 'box'
                        "center": box.xywh[0].tolist()
                    })
             
             # Draw boxes
             for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        cv2.imshow("Laptop AI Brain", display_frame)

        # 2. Run UltraDirector Logic (if active)
        director_update = {}
        if director and primary_target:
             director_update = {"status": "tracking", "target_lock": True}

        # 3. Send Insight to Cloud
        now = time.time()
        if (detections or director_update) and (now - last_send > 0.1): # Faster update (10Hz)
            payload = {
                "type": "vision_update",
                "detections": detections,
                "director": director_update,
                "timestamp": now
            }
            try:
                await websocket.send_str(json.dumps(payload))
                last_send = now
            except Exception as e:
                print(f"Send Error: {e}")
                break

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
        await asyncio.sleep(0.01)

    cap.release()
    cv2.destroyAllWindows()

async def listen_for_commands(websocket):
    """
    Listen for messages from Server (Orchestrator Commands)
    """
    async for msg in websocket:
        if msg.type == aiohttp.WSMsgType.TEXT:
            data = msg.data
            print(f"📩 Received: {data}")
            # Handle commands here...
        elif msg.type == aiohttp.WSMsgType.ERROR:
            print('ws connection closed with exception %s', websocket.exception())

async def main():
    global model
    
    print("🧠 Initializing Laptop AI (WebSocket v2)...")
    
    # 1. Load Model
    print(f"📂 Loading Model: {MODEL_PATH}")
    model = YOLO(MODEL_PATH)
    print("✅ Model Loaded")

    # 2. Connect Loop
    while True:
        try:
            print(f"🔌 Connecting to {SERVER_URL}...")
            async with aiohttp.ClientSession() as session:
                async with session.ws_connect(SERVER_URL) as websocket:
                    print("✅ Connected to Cloud!")
                    
                    # Initialize THE MASTER BRAIN (AICameraBrain)
                    # This orchestrates: Exposure, Focus, Stabilizer, Color, Scene
                    try:
                        from ai.camera_brain.laptop_ai.ai_camera_brain import AICameraBrain
                        master_brain = AICameraBrain()
                        print("✅ MASTER AI BRAIN LOADED (Orchestrating 5 Neural Engines)")
                    except ImportError as e:
                        print(f"⚠️ Master Brain not found: {e}")
                        master_brain = None
                    except Exception as e:
                         print(f"⚠️ Master Brain Init Failed: {e}")
                         master_brain = None

                    # Tone Curve is separate pipeline step usually, but Brain might manage it?
                    # Brain has .color.propose_grade but not the ACES engine.
                    # We keep GlobalToneCurve as the final "Look" applicator.
                    try:
                        from ai.camera_brain.laptop_ai.ai_pipeline.tone_curve.global_tone_curve import ToneCurveAdaptiveEngine
                        tone_engine = ToneCurveAdaptiveEngine(mode="ACES")
                        print("✅ Global Tone Curve Engine Loaded (ACES Mode)")
                    except ImportError:
                        tone_engine = None

                    # UltraDirector handles movement/pathing
                    try:
                        from ai.camera_brain.laptop_ai.ultra_director import UltraDirector
                        director = UltraDirector()
                        print("✅ UltraDirector AI Loaded (Cinematic Pathing)")
                    except ImportError:
                        director = None

                    await run_vision_loop(websocket, director, master_brain, tone_engine)
                    
        except Exception as e:
            print(f"⚠️ Connection Failed: {e}")
            print("Retrying in 5 seconds...")
            time.sleep(5)

if __name__ == "__main__":
    asyncio.run(main())
