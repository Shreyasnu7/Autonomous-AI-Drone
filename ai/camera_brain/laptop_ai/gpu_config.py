# laptop_ai/gpu_config.py
"""
GPU Configuration — Auto-detects available GPUs and sets up AI models accordingly.

TWO MODES:
  STANDARD: RTX 5070 Ti only (12GB) — lightweight models, still powerful
  ULTRA:    RTX 5090 (32GB) + RTX 5070 Ti (12GB) — maximum intelligence

The system auto-detects which mode to use at startup.
User can override via environment variable: GPU_MODE=standard or GPU_MODE=ultra
"""

import os
import torch
import logging

logger = logging.getLogger('GPUConfig')


def detect_gpu_mode():
    """
    Auto-detect available GPUs and choose the best configuration.
    Returns: 'ultra' if 5090 detected, 'standard' otherwise.
    """
    # User override
    override = os.environ.get('GPU_MODE', '').lower()
    if override in ('standard', 'ultra'):
        logger.info(f"GPU mode override: {override}")
        return override

    if not torch.cuda.is_available():
        logger.warning("No CUDA GPU detected — running CPU-only (very slow)")
        return 'cpu'

    gpu_count = torch.cuda.device_count()
    logger.info(f"Detected {gpu_count} CUDA GPU(s)")

    if gpu_count >= 2:
        # Check for 5090 (32GB) on any device
        for i in range(gpu_count):
            name = torch.cuda.get_device_name(i)
            vram_gb = torch.cuda.get_device_properties(i).total_mem / (1024**3)
            logger.info(f"  GPU {i}: {name} ({vram_gb:.1f}GB)")

            if vram_gb >= 24:  # 5090 has 32GB, any card with 24GB+ is "heavy"
                logger.info(f"✅ ULTRA MODE: Heavy GPU detected on device {i}")
                return 'ultra'

    return 'standard'


# =============================================
# STANDARD MODE — RTX 5070 Ti only (12GB)
# =============================================
STANDARD_CONFIG = {
    'mode': 'standard',
    'description': 'RTX 5070 Ti (12GB) — Lightweight but capable',

    # All models on GPU 0 (the only GPU)
    'vision_brain': {
        'model_id': 'Qwen/Qwen2.5-VL-3B-Instruct',  # 3B 4-bit (~2.4GB VRAM) — verified working (7B-4bit outputs garbage)
        'device': 'cuda:0',
        'max_frame_dim': 352,   # sharper vision (~1.9x pixels vs 256) — sees small objects/distances
        'dtype': torch.float16,
        'target_fps': 16,       # ~15-18fps at 352px on RTX 5070 Ti — still in the 10-20fps band
        'vram_estimate_gb': 2.5,
    },

    'object_detector': {
        'type': 'yolo',
        'model_path': 'yolov8n.pt',  # nano — fast, lightweight
        'device': 'cuda:0',
        'vram_estimate_gb': 0.5,
    },

    'depth_estimator': {
        'type': 'midas',
        'model_name': 'MiDaS_small',
        'device': 'cuda:0',
        'vram_estimate_gb': 0.5,
    },

    'cinematic_pipeline': {
        'device': 'cuda:0',
        'gpu_aces': True,  # GPU-accelerated tone curve
        'vram_estimate_gb': 2.0,
    },

    'predictor': None,       # No V-JEPA in standard mode
    'segmenter': None,       # No SAM 2 in standard mode
    'grounding': None,       # No Grounding DINO in standard mode

    'gemini_cloud': {
        'model': 'gemini-2.0-flash',
        'interval_seconds': 2.0,
    },

    'total_vram_estimate_gb': 7.0,  # Fits comfortably in 12GB
}


# =============================================
# ULTRA MODE — RTX 5090 (32GB) + RTX 5070 Ti (12GB)
# =============================================
ULTRA_CONFIG = {
    'mode': 'ultra',
    'description': 'RTX 5090 (32GB) + RTX 5070 Ti (12GB) — Maximum Intelligence',

    # ---- RTX 5090 (cuda:0 or whichever is the 32GB card) ----
    'vision_brain': {
        'model_id': 'Qwen/Qwen3-VL-32B-Instruct',
        'quantization': 'int4',       # AWQ or GPTQ 4-bit
        'device': 'heavy_gpu',        # Resolved at runtime to the 32GB card
        'max_frame_dim': 1024,        # Full resolution understanding
        'dtype': torch.float16,
        'target_fps': 10,             # 8-12fps sweet spot
        'vram_estimate_gb': 18.0,
        'flash_attention': True,
        'torch_compile': True,
    },

    'depth_estimator': {
        'type': 'depth_anything_v3',
        'model_name': 'depth-anything-v3-metric-large',
        'device': 'heavy_gpu',
        'target_fps': 15,
        'vram_estimate_gb': 2.0,
    },

    'predictor': {
        'type': 'vjepa2',
        'model_name': 'v-jepa-2-base',
        'device': 'heavy_gpu',
        'target_fps': 10,
        'vram_estimate_gb': 2.0,
    },

    'grounding': {
        'type': 'grounding_dino',
        'model_name': 'groundingdino-swint-ogc',
        'device': 'heavy_gpu',
        'target_fps': 30,
        'vram_estimate_gb': 3.0,
    },

    # ---- RTX 5070 Ti (cuda:1 or whichever is the 12GB card) ----
    'object_detector': {
        'type': 'rfdetr',
        'model_name': 'rfdetr-large',
        'device': 'light_gpu',        # Resolved at runtime to the 12GB card
        'target_fps': 100,
        'vram_estimate_gb': 2.0,
    },

    'segmenter': {
        'type': 'sam2',
        'model_name': 'sam2-large',
        'device': 'light_gpu',
        'target_fps': 30,
        'vram_estimate_gb': 2.0,
    },

    'cinematic_pipeline': {
        'device': 'light_gpu',
        'gpu_aces': True,
        'vram_estimate_gb': 2.0,
    },

    'gemini_cloud': {
        'model': 'gemini-2.5-flash',  # Upgraded cloud model
        'interval_seconds': 2.0,
    },

    'total_vram_5090_gb': 25.0,   # 7GB free on 5090
    'total_vram_5070ti_gb': 8.0,  # 4GB free on 5070 Ti
}


def resolve_devices(config):
    """
    Resolve 'heavy_gpu' and 'light_gpu' to actual CUDA device IDs.
    Heavy = card with most VRAM (5090), Light = the other one (5070 Ti).
    """
    if config['mode'] == 'standard':
        # Single GPU — everything on cuda:0
        return config

    if not torch.cuda.is_available() or torch.cuda.device_count() < 2:
        logger.warning("Ultra mode requested but <2 GPUs found — falling back to standard")
        return STANDARD_CONFIG

    # Find which GPU has more VRAM
    gpu_vram = []
    for i in range(torch.cuda.device_count()):
        vram = torch.cuda.get_device_properties(i).total_mem / (1024**3)
        gpu_vram.append((i, vram))

    gpu_vram.sort(key=lambda x: x[1], reverse=True)
    heavy_id = gpu_vram[0][0]  # Largest VRAM = heavy (5090)
    light_id = gpu_vram[1][0]  # Smaller VRAM = light (5070 Ti)

    heavy_device = f'cuda:{heavy_id}'
    light_device = f'cuda:{light_id}'

    logger.info(f"GPU Assignment: Heavy={heavy_device} ({gpu_vram[0][1]:.0f}GB), Light={light_device} ({gpu_vram[1][1]:.0f}GB)")

    # Replace device strings
    resolved = {}
    for key, value in config.items():
        if isinstance(value, dict) and 'device' in value:
            new_val = dict(value)
            if new_val['device'] == 'heavy_gpu':
                new_val['device'] = heavy_device
            elif new_val['device'] == 'light_gpu':
                new_val['device'] = light_device
            resolved[key] = new_val
        else:
            resolved[key] = value

    return resolved


def get_gpu_config():
    """
    Main entry point — returns the resolved GPU configuration.
    Call this once at DirectorCore startup.
    """
    mode = detect_gpu_mode()

    if mode == 'ultra':
        config = dict(ULTRA_CONFIG)
        logger.info("🚀 ULTRA MODE: Dual GPU — Maximum AI Intelligence")
    elif mode == 'standard':
        config = dict(STANDARD_CONFIG)
        logger.info("⚡ STANDARD MODE: Single GPU — Lightweight & Fast")
    else:
        config = dict(STANDARD_CONFIG)
        config['mode'] = 'cpu'
        logger.warning("⚠️ CPU MODE: No GPU — Very slow, for testing only")

    config = resolve_devices(config)

    # Print summary
    print("\n" + "=" * 60)
    print(f"  GPU CONFIG: {config['mode'].upper()} MODE")
    print(f"  {config.get('description', '')}")
    print("=" * 60)

    for key, value in config.items():
        if isinstance(value, dict) and 'device' in value:
            model = value.get('model_id') or value.get('model_name') or value.get('model_path') or value.get('type', '?')
            fps = value.get('target_fps', '?')
            vram = value.get('vram_estimate_gb', '?')
            print(f"  {key:25s} -> {model}")
            print(f"  {'':25s}   Device: {value['device']} | ~{vram}GB | {fps}fps")

    if config.get('predictor') is None:
        print(f"  {'predictor':25s} -> DISABLED (standard mode)")
    if config.get('segmenter') is None:
        print(f"  {'segmenter':25s} -> DISABLED (standard mode)")
    if config.get('grounding') is None:
        print(f"  {'grounding':25s} -> DISABLED (standard mode)")

    print("=" * 60 + "\n")

    return config


# Quick test
if __name__ == '__main__':
    config = get_gpu_config()
    print(f"\nMode: {config['mode']}")
    print(f"Vision: {config['vision_brain']['model_id']}")
    print(f"Detector: {config['object_detector'].get('model_path') or config['object_detector'].get('model_name')}")
