#!/usr/bin/env bash
# Persistent vLLM inference server for the drone pilot brain (Qwen2.5-VL-7B-Instruct-AWQ).
# Runs INSIDE WSL2 (Ubuntu-22.04). Loads the model ONCE, captures CUDA graphs ONCE,
# then serves every decision in ~0.3s over localhost:8100 (OpenAI-compatible API).
#
# WHY 7B-AWQ (proven 2026-07-05, head-to-head on real FPV frames):
#   The 7B beats the 3B (and the SpaceThinker-3B spatial LoRA) DECISIVELY on nav decisions:
#   direction-vs-most-open 44/45 (98%) vs the 3B spatial LoRA's 12/45 (27%), clean grounded
#   reasoning, and it navigated the whole room in the full-stack sim where the 3B fixated.
#   AWQ 4-bit keeps it FAST (~0.3s/decision, awq_marlin ~93 tok/s decode) and it fits 12GB.
#
# Launch from Windows with:
#   wsl.exe -d Ubuntu-22.04 bash -lc '~/.../start_vllm_pilot.sh'
# or run this script directly inside a WSL shell.
#
# Why these flags (proven on RTX 5070 Ti / Blackwell sm_120):
#   --quantization awq_marlin : AWQ 4-bit weights -> ~93 tok/s decode (2.7x vs fp8), fits 12GB
#   VLLM_ATTENTION_BACKEND=FLASH_ATTN + VLLM_USE_FLASHINFER_SAMPLER=0
#                             : avoid the flashinfer JIT build that fails on this GPU
#   --enable-prefix-caching   : the big static pilot prompt is cached -> near-zero prefill
#   --compilation-config cudagraph_mm_encoder : CUDA-graph the vision encoder (THE latency
#                             : lever for a VLM at short outputs; ~90ms/decision saved)
#   --limit-mm-per-prompt image=8 : allow up to 8 live frames/decision (MUST be >=5; the pilot
#                             : sends 5 frames/tick and vLLM 400s if the cap is below that)
set -e

export HF_HOME="${HF_HOME:-$HOME/hf_cache}"           # native-FS cache = ~15s load (not 5min)
export VLLM_USE_FLASHINFER_SAMPLER=0
export VLLM_ATTENTION_BACKEND=FLASH_ATTN
CU="$HOME/.local/lib/python3.10/site-packages/nvidia/cu13"
[ -d "$CU/bin" ] && export CUDA_HOME="$CU" && export PATH="$CU/bin:$PATH"

MODEL="Qwen/Qwen2.5-VL-7B-Instruct-AWQ"
PORT="${PILOT_PORT:-8100}"   # 8000 is taken by the drone bridge/media server; use 8100 for the pilot

echo "[vllm-pilot] starting $MODEL on :$PORT (HF_HOME=$HF_HOME)"
exec vllm serve "$MODEL" \
  --port "$PORT" \
  --quantization awq_marlin \
  --dtype float16 \
  --max-model-len 8192 \
  --gpu-memory-utilization 0.88 \
  --limit-mm-per-prompt '{"image": 8}' \
  --enable-prefix-caching \
  --mm-processor-kwargs '{"min_pixels": 14400, "max_pixels": 32400}' \
  --compilation-config '{"cudagraph_mm_encoder": true, "compile_mm_encoder": true}' \
  --served-model-name pilot
