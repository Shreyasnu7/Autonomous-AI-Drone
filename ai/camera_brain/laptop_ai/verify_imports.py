import sys
import os
import importlib

# Add parent directory to path to mimic director_core.py
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)
# Also add the grandparent to support 'camera_brain.farming' style if needed, though unlikely given the code
sys.path.append(os.path.dirname(parent_dir))

print(f"Testing imports from: {current_dir}")
print(f"Sys Path includes: {parent_dir}")

modules_to_test = [
    "laptop_ai.ai_frame_blender",
    "laptop_ai.ai_gimbal_brain",
    "laptop_ai.execution_router",
    "laptop_ai.ai_drone_stabilizer",
    "laptop_ai.motion_engine",
    "laptop_ai.mavllink_executor",
    "laptop_ai.render_master",
    "laptop_ai.shot_metadata",
    "laptop_ai.ai_shot_planner",
    "laptop_ai.safety_envelope",
    "laptop_ai.video_recorder",
    "laptop_ai.camera_director",
    "laptop_ai.deepstream_handler",
    "laptop_ai.pi0_pilot",
    "laptop_ai.ai_camera_pipeline",
    "laptop_ai.ai_exposure_engine",
    "laptop_ai.ai_scene_classifier",
    "laptop_ai.ai_subject_tracker",
    "laptop_ai.ai_autofocus",
    "laptop_ai.ai_color_engine",
    "laptop_ai.ai_deblur",
    "laptop_ai.ai_hdr_engine",
    "laptop_ai.ai_noise_reduction",
    "laptop_ai.ai_super_resolution",
    "laptop_ai.ai_depth_estimator",
    "laptop_ai.ai_stabilizer",
    "laptop_ai.camera_fusion",
    "laptop_ai.camera_manager",
]

failed = []
passed = []

print("\n=== STARTING IMPORT TESTS ===\n")

for module in modules_to_test:
    try:
        importlib.import_module(module)
        print(f"[PASS] {module} PASSED")
        passed.append(module)
    except ImportError as e:
        print(f"[FAIL] {module} FAILED: {e}")
        failed.append((module, str(e)))
    except Exception as e:
        print(f"[CRASH] {module} CRASHED during import: {e}")
        failed.append((module, str(e)))

print("\n=== SPECIALTY IMPORT TESTS ===\n")

# Test Farming Engine Path
try:
    from farming.farming_engine import FarmingEngine
    print(f"[PASS] farming.farming_engine PASSED (Direct)")
except ImportError:
    try:
        from camera_brain.farming.farming_engine import FarmingEngine
        print(f"[PASS] camera_brain.farming.farming_engine PASSED (Nested)")
    except ImportError as e:
        print(f"[FAIL] FarmingEngine FAILED: {e}")
        failed.append(("FarmingEngine", str(e)))

# Test Classes
print("\n=== CLASS INSTANTIATION TESTS ===\n")
try:
    from laptop_ai.ai_scene_classifier import SceneClassifier, AISceneClassifier
    s = SceneClassifier(use_torch=False)
    print("[PASS] SceneClassifier Instantiated")
except Exception as e:
    print(f"[FAIL] SceneClassifier Failed: {e}")
    failed.append(("SceneClassifier", str(e)))

try:
    from laptop_ai.ai_subject_tracker import AISubjectTracker, AdvancedAISubjectTracker
    t = AdvancedAISubjectTracker()
    print("[PASS] AdvancedAISubjectTracker Instantiated")
except Exception as e:
    print(f"[FAIL] AdvancedAISubjectTracker Failed: {e}")
    failed.append(("AdvancedAISubjectTracker", str(e)))

try:
    from laptop_ai.camera_fusion import CameraFusion
    from laptop_ai.ai_fusion_pipeline import AIFusionPipeline
    cf = CameraFusion()
    afp = AIFusionPipeline()
    print("[PASS] CameraFusion & AIFusionPipeline Instantiated")
except Exception as e:
    print(f"[FAIL] Fusion Failed: {e}")
    failed.append(("Fusion", str(e)))


print("\n==================================")
print(f"PASSED: {len(passed)}/{len(modules_to_test)}")
print(f"FAILED: {len(failed)}")
print("==================================")
if failed:
    sys.exit(1)
sys.exit(0)
