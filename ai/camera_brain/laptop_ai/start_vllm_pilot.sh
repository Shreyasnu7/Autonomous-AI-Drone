#!/usr/bin/env bash
# Persistent vLLM inference server for the drone pilot brain (Qwen2.5-VL-3B).
# Runs INSIDE WSL2 (Ubuntu-22.04). Loads the model ONCE, captures CUDA graphs ONCE,
# then serves every decision in ~0.3-0.4s over localhost:8000 (OpenAI-compatible API).
#
# Launch from Windows with:
#   wsl.exe -d Ubuntu-22.04 bash -lc '~/.../start_vllm_pilot.sh'
# or run this script directly inside a WSL shell.
#
# Why these flags (proven on RTX 5070 Ti / Blackwell sm_120):
#   --quantization fp8        : FP8 weights -> ~97 tok/s (Blackwell native FP8 cores)
#   VLLM_ATTENTION_BACKEND=FLASH_ATTN + VLLM_USE_FLASHINFER_SAMPLER=0
#                             : avoid the flashinfer JIT build that fails on this GPU
#   --enable-prefix-caching   : the big static pilot prompt is cached -> near-zero prefill. CRITICAL:
#                               local_er_brain sends that prompt as a SYSTEM message BEFORE the frames
#                               so it's a stable cacheable prefix. If it ever moves AFTER the (changing)
#                               images, caching breaks and latency ~3x's (measured 2233ms vs 699ms).
#   --compilation-config cudagraph_mm_encoder/compile_mm_encoder : CUDA-graph the vision encoder. Without
#                               it the mm-encoder runs eager = ~100ms/req extra fixed cost (measured).
#   --limit-mm-per-prompt image=8 : allow up to 8 live frames per decision
set -e

export HF_HOME="${HF_HOME:-$HOME/hf_cache}"           # native-FS cache = ~15s load (not 5min)
export VLLM_USE_FLASHINFER_SAMPLER=0
export VLLM_ATTENTION_BACKEND=FLASH_ATTN
CU="$HOME/.local/lib/python3.10/site-packages/nvidia/cu13"
[ -d "$CU/bin" ] && export CUDA_HOME="$CU" && export PATH="$CU/bin:$PATH"

MODEL="Qwen/Qwen2.5-VL-3B-Instruct"
PORT="${PILOT_PORT:-8100}"   # 8000 is taken by the drone bridge/media server; use 8100 for the pilot

echo "[vllm-pilot] starting $MODEL on :$PORT (HF_HOME=$HF_HOME)"
exec vllm serve "$MODEL" \
  --port "$PORT" \
  --quantization fp8 \
  --dtype bfloat16 \
  --max-model-len 8192 \
  --gpu-memory-utilization 0.85 \
  --limit-mm-per-prompt '{"image": 8}' \
  --enable-prefix-caching \
  --mm-processor-kwargs '{"min_pixels": 14400, "max_pixels": 32400}' \
  --compilation-config '{"cudagraph_mm_encoder": true, "compile_mm_encoder": true}' \
  --served-model-name pilot
# NOTE: tried --structured-outputs-config xgrammar+disable_any_whitespace to force compact (no-space)
# JSON — on this vLLM 0.23 build it did NOT strip whitespace AND forced the slower xgrammar backend
# (~390ms vs ~324ms). Left OUT. json_object (auto backend) in _infer_server is faster; the model is
# compact most of the time and the robust parser handles the rest.
