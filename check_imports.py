import sys
import os

# Add correct parent directory to path
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "ai", "camera_brain"))

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
    "laptop_ai.ai_colour_engine",
    "laptop_ai.ai_colourist",
    "laptop_ai.ai_deblur",
    "laptop_ai.ai_hdr_engine",
    "laptop_ai.ai_noise_reduction",
    "laptop_ai.ai_super_resolution",
    "laptop_ai.ai_superres",
    "laptop_ai.ai_depth_estimator",
    "laptop_ai.ai_lensfix",
    "laptop_ai.ai_stabilizer",
    "laptop_ai.ai_motion_blur_controller",
    "laptop_ai.ai_video_engine",
    "laptop_ai.ai_fusion_pipeline",
    "laptop_ai.motion_curve",
    "laptop_ai.flow_field",
    "laptop_ai.obstacle_warp",
    "laptop_ai.camera_manager",
    "laptop_ai.pi_camera",
    "laptop_ai.pi_camera_driver",
    "laptop_ai.threaded_camera",
    "laptop_ai.exposure_tools",
    "laptop_ai.pipeline_assembler",
    "laptop_ai.sort_tracker",
    "laptop_ai.follow_brain",
    "laptop_ai.ai_auto_editor"
]

print("--- STARTING IMPORT CHECK ---")
for mod in modules_to_test:
    try:
        __import__(mod)
        print(f"[OK] {mod}")
    except ImportError as e:
        print(f"[FAIL] {mod} FAILED: {e}")
    except Exception as e:
        print(f"[CRASH] {mod} CRASHED: {e}")
print("--- FINISHED IMPORT CHECK ---")
