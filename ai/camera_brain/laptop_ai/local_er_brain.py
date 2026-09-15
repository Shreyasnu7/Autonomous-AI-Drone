import cv2
import os
import math
import time
import json
import base64
import urllib.request
import urllib.error
import numpy as np
import threading
import logging

# Heavy GPU libs (torch/transformers) are only needed for the SLOW in-process fallback.
# Import lazily so the fast vLLM-server path doesn't pull ~3GB of CUDA libs into Windows.
torch = None
Qwen2_5_VLForConditionalGeneration = None
AutoProcessor = None
process_vision_info = None

def _load_local_inference_libs():
    """Import transformers/torch only if we must run the slow in-process fallback."""
    global torch, Qwen2_5_VLForConditionalGeneration, AutoProcessor, process_vision_info
    if torch is not None:
        return
    import torch as _torch
    from transformers import Qwen2_5_VLForConditionalGeneration as _Q, AutoProcessor as _AP
    from qwen_vl_utils import process_vision_info as _pvi
    torch = _torch
    Qwen2_5_VLForConditionalGeneration = _Q
    AutoProcessor = _AP
    process_vision_info = _pvi

logger = logging.getLogger('LocalERBrain')

import re as _re

# Strict output schema for guided decoding — the exact compact flight command the executor parses.
# With the server's xgrammar backend + disable_any_whitespace this emits e.g.
# {"flight":{"vx":0.3,"vy":0,"vz":0,"yaw_rate":0}} with NO whitespace (~9-11 tokens = fastest).
_FLIGHT_SCHEMA = {
    "name": "flight_command",
    "schema": {
        "type": "object",
        "properties": {
            "flight": {
                "type": "object",
                "properties": {
                    "vx": {"type": "number"}, "vy": {"type": "number"},
                    "vz": {"type": "number"}, "yaw_rate": {"type": "number"},
                },
                "required": ["vx", "vy", "vz", "yaw_rate"],
                "additionalProperties": False,
            },
        },
        "required": ["flight"],
        "additionalProperties": False,
    },
}

def _flight_sig(fl):
    """Rounded (vx,vy,vz,yaw) signature of a flight command."""
    try:
        return tuple(round(float((fl or {}).get(k, 0) or 0), 2) for k in ('vx', 'vy', 'vz', 'yaw_rate'))
    except Exception:
        return (0.0, 0.0, 0.0, 0.0)

def _sig_near(a, b, eps=0.06):
    """True if two flight signatures are within eps on every axis (yaw axis scaled up)."""
    if not a or not b:
        return False
    for i, (x, y) in enumerate(zip(a, b)):
        tol = eps * (100 if i == 3 else 1)   # yaw_rate is deg/s -> a looser absolute tolerance
        if abs(x - y) > tol:
            return False
    return True

def _norm_reason(s):
    """Normalize a reasoning string so trivial wording changes collapse to the same rut key."""
    return _re.sub(r'[^a-z0-9]+', ' ', str(s or '').lower()).strip()

# ── HYBRID INTENT CONTROL ────────────────────────────────────────────────────────────────────
# The say-do gap (proven 2026-07-05: the 7B REASONS right but EMITS wrong velocity numbers — it
# commanded BACKWARD into a wide-open front on 18/20 real frames = 5% correct). The fix: the VLM
# outputs only a DIRECTION word (its reliable strength, ~98%), and CODE converts that direction into
# the exact clearance-safe velocity (the geometry it fails at) -> 100% correct on the same frames.
_HEAD_AZ = {                              # body-frame AZIMUTH in degrees (0=fwd, +=right, spherical)
    "fwd": 0.0, "fwd_right": 45.0, "right": 90.0, "back_right": 135.0,
    "back": 180.0, "back_left": -135.0, "left": -90.0, "fwd_left": -45.0,
    # Synonyms. An unrecognised head word falls through to the hover branch, so a model that
    # said "forward" instead of "fwd" stopped the aircraft dead with no warning -- the failure
    # is silent and indistinguishable from a chosen hover. _CLIMB_ELEV already accepted
    # synonyms; this map accepted none.
    "forward": 0.0, "ahead": 0.0, "front": 0.0, "f": 0.0,
    "forward_right": 45.0, "front_right": 45.0, "diag_right": 45.0,
    "forward_left": -45.0, "front_left": -45.0, "diag_left": -45.0,
    "backward": 180.0, "backwards": 180.0, "behind": 180.0, "reverse": 180.0, "b": 180.0,
    "backward_right": 135.0, "backward_left": -135.0,
    "r": 90.0, "starboard": 90.0, "l": -90.0, "port": -90.0,
}
_CLIMB_ELEV = {                           # ELEVATION angle in degrees = the flight SLOPE (up=+, dive=-)
    "dive_steep": -25.0, "dive": -12.0, "descend": -12.0, "down": -12.0,
    "level": 0.0, "hold": 0.0, "": 0.0,
    "climb": 12.0, "up": 12.0, "ascend": 12.0, "climb_steep": 25.0,
}
_PACE = {"creep": 0.45, "slow": 0.6, "cruise": 1.0, "normal": 1.0, "fast": 1.0, "": 1.0}

def _num(v, default):
    try:
        f = float(v)
        return default if f in (9999.0, -1.0) else f
    except Exception:
        return default

def _speed_cap_cm(cm):
    """Stopping-distance table (same as director_core._clearance_speed_cap): clearance cm -> max m/s."""
    return 0.0 if cm < 60 else 0.20 if cm < 100 else 0.35 if cm < 150 else 0.50 if cm < 250 else 0.60

def _drone_limits(sensors):
    """This DRONE's real flight envelope, read LIVE from the FC (director_core fills sensors['capabilities']
    from ArduPilot params ANGLE_MAX/PILOT_SPEED_UP/PILOT_SPEED_DN/WPNAV_SPEED via
    DroneConfig.update_from_fc_telemetry each loop). So the movement stays inside what THIS build can do.
    WIND / GUSTS / STABILISATION are NOT estimated here — that is ArduPilot's job: its position controller
    (GUIDED / Loiter / PosHold, with GPS) actively rejects wind and holds the AI's commanded velocity/
    position. The AI just picks the intent + the right FC MODE; the FC copes with the air. Defaults are a
    pre-FC-connect bootstrap only."""
    c = (sensors or {}).get('capabilities') or {}
    return {
        "vmax":    float(c.get('max_horiz_speed_ms') or 6.0),   # WPNAV_SPEED (m/s) — the drone's ceiling
        "climb":   float(c.get('max_climb_ms') or 5.0),         # PILOT_SPEED_UP
        "descent": float(c.get('max_descent_ms') or 1.5),       # PILOT_SPEED_DN
        "yaw":     float(c.get('max_yaw_degs') or 60.0),        # MAX_YAW_RATE
    }

def _resolve_yaw(yaw_mode, az, subject_bearing, yaw_max=60.0):
    """CODE computes the PRECISE yaw the AI's intent implies — the AI never emits a yaw NUMBER (say-do),
    it picks a yaw BEHAVIOUR and code makes it exact, CLAMPED to THIS drone's real max yaw rate (from the
    FC). This is what lets cinematic moves EMERGE with NO hardcoded maneuver: 'move sideways' +
    'face_subject' each tick == a smooth ORBIT. face_travel=turn toward travel; face_subject=centre the
    tracked subject; scan_left/right=search sweep; hold=no yaw."""
    ym = str(yaw_mode or "face_travel").lower().strip()
    ymax = max(10.0, float(yaw_max))
    if ym == "face_subject" and subject_bearing is not None:
        return round(max(-ymax, min(ymax, float(subject_bearing) * 1.5)), 1)
    if ym == "scan_left":  return -min(20.0, ymax)
    if ym == "scan_right": return min(20.0, ymax)
    if ym == "hold":       return 0.0
    return round(max(-ymax, min(ymax, az * 1.2)), 1)      # face_travel (default)

def _resolve_intent(head, climb, sensors, pace="cruise", yaw_mode="face_travel", subject_bearing=None):
    """HYBRID + FULL SPHERICAL: turn the AI's (direction, slope, pace, yaw-behaviour) INTENT into a precise
    clearance-safe 3D body-velocity + yaw. The AI picks WHERE/how (categorical, reliable); CODE does the
    exact trig + speed + yaw (the numbers the AI gets wrong). NO maneuver is hardcoded — orbits/arcs/follows
    EMERGE from the AI's own per-tick intent (e.g. head='right' + yaw='face_subject' each tick == orbit).
        azimuth θ (head), elevation φ (climb slope), speed s (clearance*pace), yaw from the yaw-behaviour.
        vx = s·cosφ·cosθ (fwd+)   vy = s·cosφ·sinθ (right+)   vz = s·sinφ (up+ = the slope)."""
    head = str(head or "hover").lower().strip()
    climb = str(climb or "level").lower().strip()
    pace = str(pace or "cruise").lower().strip()
    elev = _CLIMB_ELEV.get(climb, 0.0)
    lim = _drone_limits(sensors)          # THIS drone's live FC envelope (speed/climb/descent/yaw)
    def _vz_clamp(v): return max(-lim["descent"], min(lim["climb"], v))
    az = _HEAD_AZ.get(head)
    if az is None and head not in ("hover", "stop", "hold", "", "none"):
        # A deliberate hover is expected; an unrecognised direction is not. Say so, because the
        # resulting dead stop is indistinguishable from the pilot choosing to hold.
        logger.warning(f"unknown head direction {head!r} -> treating as hover")
    if az is None:                        # hover/stop -> hold position, but still allow a vertical slope
        vz = _vz_clamp(0.30 * (1 if elev > 0 else (-1 if elev < 0 else 0)))  # capped to the drone's climb/descent
        return {"vx": 0.0, "vy": 0.0, "vz": round(vz, 3),
                "yaw_rate": _resolve_yaw(yaw_mode, 0.0, subject_bearing, lim["yaw"])}
    F = _num(sensors.get('t1', sensors.get('tof_front')), 400)
    R = _num(sensors.get('t2', sensors.get('tof_right')), 400)
    B = _num(sensors.get('t3', sensors.get('tof_back')), 400)
    L = _num(sensors.get('t4', sensors.get('tof_left')), 400)
    clr = F if abs(az) <= 45 else (R if 45 < az <= 135 else (L if -135 <= az < -45 else B))
    # DIRECTION SAFETY-CORRECTION (the deterministic Part-B guard): the AI's chosen direction is only a
    # PREFERENCE. If it is BLOCKED (<80cm), CODE re-steers to the MOST-OPEN sector instead of stalling in
    # front of the obstacle — so a wrong/forward-biased head can never drive us into it AND we still make
    # progress around it. This REFINES the AI's intent; it does not dictate a maneuver. Only boxed -> hover.
    if clr < 80:
        sectors = {0.0: F, 90.0: R, -90.0: L, 180.0: B}
        # Prefer the MOST-FORWARD open sector (not just the max-clearance one) so we make forward
        # progress / turn instead of reflexively BACKING UP. Sort open sectors by |azimuth| then clearance.
        _open = sorted(((abs(a), -c, a, c) for a, c in sectors.items() if c >= 120))
        if _open:
            az, clr = _open[0][2], _open[0][3]
        else:
            return {"vx": 0.0, "vy": 0.0,
                    "vz": round(_vz_clamp(0.30 if elev > 0 else (-0.30 if elev < 0 else 0.0)), 3),
                    "yaw_rate": _resolve_yaw(yaw_mode, 0.0, subject_bearing, lim["yaw"])}
    # horizontal speed = the clearance stopping-distance cap (indoor safety) scaled by pace, but NEVER
    # beyond what THIS drone can do (its FC WPNAV_SPEED ceiling). Indoors the clearance binds; in open
    # space the drone's real max applies. Wind/gusts are rejected by the FC's mode, not compensated here.
    speed = min(_speed_cap_cm(clr) * _PACE.get(pace, 1.0), lim["vmax"])
    ar, er = math.radians(az), math.radians(elev)
    ch = math.cos(er)
    return {
        "vx": round(speed * ch * math.cos(ar), 3),   # forward
        "vy": round(speed * ch * math.sin(ar), 3),   # right
        "vz": round(_vz_clamp(speed * math.sin(er)), 3),   # up — the slope's rate, capped to this drone's climb/descent
        "yaw_rate": _resolve_yaw(yaw_mode, az, subject_bearing, lim["yaw"]),
    }

def _parse_decision(raw):
    """Robustly extract the decision dict from the model's raw text. A 3B often gets CLIPPED at the
    token cap MID-OBJECT (reasoning-first means the flight numbers come last, so they're what's lost) —
    a naive json.loads then fails and the decision is silently dropped, leaving the drone on a stale
    hover. So: strip fences, take from the first '{', and try progressively repairing a truncated tail
    by appending the missing closing quote/braces. Returns {} only if nothing usable is recoverable."""
    if not raw:
        return {}
    s = str(raw)
    if "```json" in s:
        s = s.split("```json", 1)[1]
    s = s.replace("```", "")
    i = s.find("{")
    if i < 0:
        return {}
    s = s[i:]
    for cand in (s, s + "}", s + '"}', s + "}}", s + '"}}', s + '0}}', s + '0}}}', s + '"}}}'):
        try:
            d = json.loads(cand)
            if isinstance(d, dict):
                return d
        except Exception:
            pass
    m = _re.search(r'\{.*\}', s, _re.DOTALL)
    if m:
        try:
            return json.loads(m.group(0))
        except Exception:
            pass
    return {}

# ER Brain Prompt - Continuous autonomous reasoning at 20-30 FPS
ER_BRAIN_PROMPT = """You are the Local Embodied Reasoning (ER) Brain of an advanced cinematic drone.
You run locally at 20-30 FPS. The Cloud Director (Gemini) gives you a high-level intent every ~2 seconds.
Between updates, YOU are the pilot. You must CONTINUOUSLY pursue the Director's intent using your own
spatial reasoning — like a real pilot who was given a mission briefing and now flies autonomously.

== YOUR ROLE — you are the REASONING PILOT BRAIN ==
- The Director (Gemini) hands you a DETAILED plan (GOAL + DIRECTOR PLAN + STEPS). You are the live
  brain that EXECUTES it: reason through that plan step-by-step, FUSING the camera image + LiDAR +
  ToF + depth + your own last decisions, and make EVERY real-time decision yourself.
- The Director says WHAT to achieve; YOU decide HOW, frame-by-frame: velocity, heading, gimbal,
  obstacle avoidance — AND which step of the plan you are on right now.
- You NEVER stop and wait for the next Director update. You keep flying the mission, advancing through
  its steps.
- SPEED MATTERS: you run continuously at 10-20+ FPS, so keep your JSON OUTPUT TINY. "reasoning" = ONE
  terse line (which step + the decision). Rich INPUT is fine; a long output slows EVERY frame.
- Use your PREVIOUS DECISIONS as momentum — maintain smooth continuous motion unless obstacles or
  scene changes require adjustment. A real pilot doesn't jerk to a halt every 2 seconds.

== SCENE UNDERSTANDING & COMPLEX TASK REASONING (you handle non-trivial tasks, not just go/stop) ==
- FIRST build a mental model of the scene from the FUSED video + sensors: what OBJECTS are present (use
  the detections — label, distance, bearing), the room LAYOUT, where the FREE space, openings, walls, and
  the TARGET are. The overlaid depth + 360° LiDAR + ToF give you real geometry — actually use it.
- IDENTIFY what the goal needs (a specific object / person / colour / place / direction). If the target
  is NOT in view, SEARCH for it — yaw to scan the room, or reposition for a better angle. Don't give up.
- DECOMPOSE the goal into concrete sub-steps and TRACK which one you're on; finish a step then advance
  (e.g. "find the doorway" → "approach it" → "pass through" → "scan the next area"). Multi-step is normal.
- PLAN A PATH that makes real progress toward the current sub-goal while staying clear: estimate the
  distance/direction to the target from depth + detections + LiDAR and head that way deliberately.
- ADAPT: if blocked or the scene changes, RE-ROUTE to still achieve the goal — never just stop and idle.
- VERIFY progress from the live video (getting closer? target centred? step complete?) and correct.

== CONTINUITY RULES ==
- If you were moving forward at 2m/s to track a subject, KEEP moving forward unless something changed
- Smoothly adjust velocity, don't jump between extremes
- If the Director hasn't updated recently, EXTRAPOLATE — continue the trajectory and intent
- Your reasoning should explain what you're doing to pursue the Director's goal

== PRECISION & GROUNDING — NON-NEGOTIABLE (you pilot an EXPENSIVE drone; 1 wrong move = a crash) ==
You are the ULTIMATE pilot — you command everything. But you must NEVER GUESS a number. EVERY velocity
you output must be DERIVED from a SPECIFIC measured sensor value, by the rules below. If you cannot tie a
number to a reading, do NOT command it — slow down or hover and re-observe.

DERIVE SPEED FROM CLEARANCE (look it up — do NOT invent it). For the direction you want to move, read the
clearance in THAT direction (ToF F/B/L/R in cm, or the LiDAR sector / NEAREST-OBSTACLE for diagonals).
Then the MAX speed you may command in that direction is:
  • clearance < 60 cm   → 0.0 m/s   (FORBIDDEN to move that way — too close to stop in time)
  • 60–100 cm           → ≤ 0.20 m/s
  • 100–150 cm          → ≤ 0.35 m/s
  • 150–250 cm          → ≤ 0.50 m/s
  • > 250 cm            → ≤ 0.80 m/s (and never above DRONE LIMITS max speed)
This is your stopping-distance guarantee (a quad brakes at ~2.5 m/s²; standoff = 2× rotor arm = 45 cm).
NEVER exceed the row that matches the clearance in your chosen direction.

HARD NO-GO RULES (these override the mission — obstacle avoidance ALWAYS wins):
- If front clearance < 60 cm: vx MUST be ≤ 0. Do not creep forward into it. Instead YAW toward, or strafe
  (vy) into, the MOST-OPEN sector (pick the direction with the LARGEST measured clearance).
- Same rule for every axis: never command a velocity COMPONENT toward any sector closer than its table speed.
- vz (climb/descend) only within altitude limits; the horizontal LiDAR does NOT clear vertical — be cautious
  up/down and use the vertical scan below before committing to a climb/descent.
- Start of every move: EASE IN (ramp up); approaching the target/an obstacle: EASE OUT (ramp down) so you
  STOP on target without overshoot. Never step-input.

ORIENTATION IS NOT GUESSED — IT IS FC GROUND TRUTH. You do NOT compute roll/pitch. You command a DESIRED
yaw_rate (deg/s) and body velocities; ArduPilot's stabilized controller achieves the exact attitude. Read
the live roll/pitch/yaw only to understand your current heading + correct drift — never to fabricate angles.

CONFIDENCE: output a confidence 0.0–1.0. If the scene is unclear, sensors conflict, or you're unsure where
the target/free space is → confidence LOW → command SLOW (halve the table speed) or hover and re-observe.
High confidence is required before any fast move. A real pilot never commits fast on uncertainty.

== SENSOR PRIORITY ==
- < 1 m: TRUST the VL53L1X ToF + YDLidar (cm-level) over the camera/depth estimate.
- 1–10 m: use depth + spatial grid + detections for planning, confirmed by LiDAR.
- ALWAYS prioritize obstacle avoidance over intent execution.

== HARDWARE / PHYSICS — fly within what the airframe can ACTUALLY do ==
- The sensor data gives you FLIGHT DYNAMICS + DRONE LIMITS (live, from the flight controller). RESPECT
  them: never command a lean / climb / descent / speed beyond DRONE LIMITS. Derive your hover effort
  from the live throttle, and your thrust HEADROOM from cell voltage + throttle — if voltage sags or
  throttle is already high, EASE OFF (slower, gentler); lift is limited.
- The drone has MASS + MOMENTUM: ramp velocity up then DOWN (ease-in/ease-out), never step-input, so a
  move ENDS on target without overshoot. Read trim (roll/pitch) and correct drift smoothly.
- Gentle is COMPULSORY right after takeoff and before landing; in steady flight use the full envelope.
- You are reasoning on a small/agile quad indoors — think about clearance, stopping distance (grows
  with speed²), and the room geometry before every move.

== ACTIVE VERTICAL PERCEPTION (only YOU can do this) ==
- The YDLidar scans a 360 HORIZONTAL plane only — it is BLIND above and below that height. The Director
  cannot bob the airframe; only you, flying frame-by-frame, can. So actively SAMPLE the vertical dimension.
- ONLY when AIRBORNE. Never bob or tilt on the ground.
- Before entering a NEW area, BEFORE any climb/descent, and whenever vertical clearance is uncertain
  (ESPECIALLY in blind mode / no camera): do a quick reconnaissance SWEEP toward where you're about to
  go — give a brief vx/vy NUDGE in that direction so the airframe pitches DOWN-toward-target (this aims
  the camera + tips the 360° lidar plane down to catch FLOOR-level obstacles on your path), then ease
  back UP and settle (tip the plane up to catch OVERHEAD hazards). Pair it with a small vz bob (~0.1-0.2m).
  One smooth down-then-up scan along the intended path reveals low furniture, table edges, steps, ceilings,
  shelves and hanging wires that sit above/below the flat scan plane — fuse what it reveals BEFORE committing.
- Keep it GENTLE and brief — a reconnaissance wobble, never a lurch — and stay within safe altitude
  (don't rise into a ceiling or sink to the floor). Fuse what the bob reveals into your next move.
- Express the bob through vz (up/down) and a brief vx/vy nudge in your flight output; it must stay smooth.

== OUTPUT FORMAT — ONE TINY COMPACT JSON LINE (output tokens are the ONLY latency cost) ==
LATENCY FACT: everything above (frames + sensors + this whole briefing) is prefill and is nearly FREE
(~35ms, prefix-cached). The ONLY thing that costs time is the tokens you OUTPUT (~112 tok/s). So the
decision must be a MINIMAL compact control JSON — every extra key/word directly slows the flight loop.
Output ONLY this one compact line of raw JSON, nothing else — NO markdown, NO ``` fence, NO prose, NO
"reasoning"/"confidence"/"obstacle_alert" fields, NO spaces. Use a bare 0 for any zero axis. SCHEMA:
{"flight":{"vx":<fwd m/s>,"vy":<right m/s>,"vz":<up m/s>,"yaw_rate":<deg/s>}}
UNITS the FC executes directly: vx/vy/vz in m/s (vx=forward+, vy=right+, vz=up+), yaw_rate in deg/s.
Each number MUST obey the clearance→speed table + HARD NO-GO rules above — COMPUTE it from the actual
clearance reading, don't default to a round number. When the front is blocked (<60cm) set vx 0 and YAW
(yaw_rate) or STRAFE (vy) toward the largest clearance — never just idle. A CLEAR direction with real
clearance (>150cm open) MUST get a real nonzero speed per the table; only command all-zeros (hover) when
EVERY direction is <60cm. Just the single tiny JSON object — no other keys, no text, no markdown.
"""

# CINEMATIC MODE = HIDDEN POTENTIAL. OFF by default (env CINEMATIC_MODE=1 to switch on) — kept SEPARATE
# so the lean 3B core stays focused on simple, fast flight NOW. Switch it on later (with upgraded
# hardware / bigger model) and the drone frames shots like a director of photography. Appended to the
# pilot prompt only when enabled.
CINEMATIC_FRAMING_PROMPT = """
== CINEMATIC FRAMING & GIMBAL CRAFT (cinematographer mode) ==
- Command the GIMBAL (pitch, yaw) AND the drone TOGETHER — compose the shot every frame.
- KEEP THE SUBJECT FRAMED: use detections (box + bearing + distance) for rule-of-thirds placement,
  correct HEADROOM, and LEAD ROOM ahead of the subject's motion/look. Correct drift smoothly (gimbal
  first, then a gentle reposition); never let the subject clip out of frame.
- GIMBAL PITCH = emotion: -=look DOWN (high/top-down), 0=level eye-line, +=look UP (low-angle/heroic).
- COORDINATE move+gimbal per shot: ORBIT→yaw to hold subject centred; PUSH-IN→keep centred closing in;
  REVEAL→start tight/low then tilt up while rising; TRACK/FOLLOW→match speed + keep lead room.
- SMOOTH & EASED motion (ease-in/out), gimbal matched to the drone — cinematic, never jerky. Reframe
  around obstacles, don't abandon the shot.
"""

class LocalERBrain:
    def __init__(self):
        self.connected = False
        self.model_id = "Qwen/Qwen2.5-VL-3B-Instruct"

        # ── FAST PATH: vLLM server (WSL2) ───────────────────────────────────────────────
        # Inference runs in a persistent vLLM server (see start_vllm_pilot.sh) reachable over
        # localhost. Same model + GPU, but ~97 tok/s vs ~13 in-process -> ~0.3-0.4s/decision.
        # If the server isn't up, connect() falls back to the slow in-process transformers path.
        self.mode = "server"  # set to "local" by connect() if the server is unreachable
        self.server_url = os.getenv("PILOT_SERVER_URL", "http://127.0.0.1:8100")
        self._served_name = os.getenv("PILOT_SERVED_NAME", "pilot")
        self._http_timeout = float(os.getenv("PILOT_HTTP_TIMEOUT", "8"))

        # RTX 5070 Ti can handle 20-30 FPS with 3B model at 256px
        self.max_frame_dim = 352   # sharper vision (overridden by gpu_config vision_brain)
        self.min_interval = 0.0    # don't self-throttle; the model's own latency sets the rate
        # FRAMES PER DECISION — frames are nearly FREE (prefill is cheap + prefix-cached), so we send a
        # short live-video window for real motion understanding. Tune via PILOT_FRAMES.
        self._video_frames = int(os.getenv("PILOT_FRAMES", "5"))
        # OUTPUT TOKEN CAP — THE real latency lever. MEASURED on the server: latency ≈ 35ms (5 frames,
        # prefill, cached prompt) + output_tokens/~112 tok/s. So a compact flight-only JSON (~18 tok)
        # ⇒ ~0.2s WITH 5 frames; a bloated 40-tok object ⇒ ~0.46s. The schema is now flight-only, so 40
        # is ample headroom to close it (json_object mode guarantees validity anyway). Tune PILOT_MAX_TOKENS.
        self.max_new_tokens = int(os.getenv("PILOT_MAX_TOKENS", "40"))
        # Visual TOKEN budget per frame. Cap pixels so several frames stay cheap; exact numbers also
        # arrive as TEXT, so the image only needs spatial grounding, not fine print.
        self.max_pixels = int(os.getenv("PILOT_MAX_PIXELS", str(200 * 200)))
        self.min_pixels = 120 * 120
        self._jpeg_quality = int(os.getenv("PILOT_JPEG_Q", "75"))
        self._last_infer_s = 0.0
        self._infer_count = 0
        # HIDDEN POTENTIAL: cinematic shot-framing is a SEPARATE capability, OFF now (3B does simple
        # flight). Flip CINEMATIC_MODE=1 later (with upgraded hardware) to let it frame shots like a DP.
        self.cinematic_mode = (os.getenv("CINEMATIC_MODE", "0") == "1")
        
        # State
        self.last_api_call_time = 0
        self.current_director_intent = "Hover and wait for instructions."
        self.director_intent_time = 0  # When Gemini last updated
        self._input_queue = []
        self._latest_decision = None
        self._decision_history = []  # Last N decisions for trajectory memory
        self._max_history = 5
        # ── ANTI-DEGENERATION SAMPLING (the ROOT-CAUSE fix, always on) ──────────────────
        # The proven repeat-for-20-50-frames failure is TEXT DEGENERATION: greedy decoding (temperature=0)
        # makes each repeated token MORE likely next step — a self-reinforcing loop (Holtzman et al. 2020,
        # "The Curious Case of Neural Text Degeneration"). The correct fix is at the DECODER, not a
        # hardcoded escape maneuver: sample with a small temperature + nucleus (top_p) and PENALIZE
        # repetition/frequency/presence so the model itself won't loop. This lets it keep THINKING (it
        # still grounds numbers in the clearance table) instead of us dictating its move.
        # RECIPE: a small temperature + nucleus top_p + a MILD repetition_penalty break the greedy loop
        # WITHOUT corrupting the short JSON. NOTE: strong frequency/presence penalties wreck a tiny
        # structured output (they penalize the digits/keys the JSON NEEDS -> empty/garbage reasoning),
        # so we keep them at 0 and rely on temperature + repetition_penalty, which don't distort it.
        self._base_temperature = float(os.getenv("PILOT_TEMP", "0.35"))    # >0 breaks greedy degeneration
        self._temperature = self._base_temperature                        # rises further only while stuck
        self._top_p = float(os.getenv("PILOT_TOP_P", "0.9"))              # nucleus sampling
        self._repetition_penalty = float(os.getenv("PILOT_REP_PEN", "1.15"))   # >1 discourages reused tokens
        self._frequency_penalty = float(os.getenv("PILOT_FREQ_PEN", "0.0"))    # 0: corrupts short JSON
        self._presence_penalty = float(os.getenv("PILOT_PRES_PEN", "0.0"))     # 0: corrupts short JSON
        # Guided JSON output: guarantees a valid, fence-free, parseable object BUT ~halves gen speed.
        # Default ON (safety); set PILOT_JSON_MODE=0 for raw speed (robust parser then handles validity).
        self._json_mode = (os.getenv("PILOT_JSON_MODE", "1") == "1")
        # HYBRID INTENT CONTROL (DEFAULT ON): the VLM outputs a DIRECTION word, CODE computes the safe
        # velocity — closes the say-do gap (raw-velocity output = 5% correct / backs into open space;
        # hybrid = 100% correct on real frames). Set PILOT_HYBRID=0 to revert to raw-velocity output.
        self._hybrid = (os.getenv("PILOT_HYBRID", "1") == "1")
        # FIXATION DETECTOR — now only a SAFETY NET on top of the sampling fix. If the model STILL fuzzily
        # repeats (a) near-identical flight commands, or (b) the same reasoning sentence, we (1) escalate
        # the sampling knobs harder and (2) tell it to RE-OBSERVE and decide fresh — WITHOUT dictating the
        # answer (it must read the live clearances and choose the direction itself = it keeps reasoning).
        self._repeat_count = 0
        self._reason_repeat = 0
        self._recent_sigs = []           # rolling window of recent flight signatures (fuzzy fixation)
        self._recent_reasons = []        # rolling window of recent normalized reasoning strings (rut)
        self._blocked_hint = None        # set by director_core.note_blocked() when avoidance clamps us
        self._blocked_hint_time = 0.0
        self._lock = threading.Lock()
        
        self.worker_thread = threading.Thread(target=self._er_loop, daemon=True)
        
    def connect(self):
        # ── FAST PATH FIRST: is the vLLM pilot server already up? ──────────────────────
        if self._ping_server():
            self.mode = "server"
            self.connected = True
            self.worker_thread.start()
            logger.info(f"✅ Local ER Brain Online (vLLM server @ {self.server_url}): "
                        f"{self._video_frames} frames/decision, ~0.3-0.4s, no Windows VRAM used.")
            print(f"✅ Pilot using FAST vLLM engine @ {self.server_url} "
                  f"({self._video_frames} live frames, out cap {self.max_new_tokens} tok).")
            return True
        logger.warning(f"⚠️ vLLM pilot server not reachable at {self.server_url} — falling back to the "
                       f"SLOW in-process engine. Start it with start_vllm_pilot.sh for ~0.4s decisions.")
        print(f"⚠️ FAST engine offline ({self.server_url}). Falling back to slow in-process Qwen. "
              f"Run: wsl bash laptop_ai/start_vllm_pilot.sh")
        self.mode = "local"
        return self._connect_local()

    def _ping_server(self):
        """True if the vLLM server answers /v1/models within a short timeout."""
        try:
            req = urllib.request.Request(f"{self.server_url}/v1/models", method="GET")
            with urllib.request.urlopen(req, timeout=2.5) as r:
                return r.status == 200
        except Exception:
            return False

    def _connect_local(self):
        _load_local_inference_libs()
        logger.info(f"Loading {self.model_id} in bf16 into VRAM... (RTX 5070 Ti, ~7GB)")
        import platform
        from transformers import BitsAndBytesConfig
        # SDPA (PyTorch scaled-dot-product attention) is FAR faster than eager and works on Windows
        # WITHOUT flash-attn — big latency win. Linux w/ flash_attn installed uses the fastest path.
        attn_impl = "sdpa"
        if platform.system() != "Windows":
            try:
                import flash_attn
                attn_impl = "flash_attention_2"
            except ImportError:
                attn_impl = "sdpa"
        logger.info(f"Attention impl: {attn_impl}")

        # Try 1: bfloat16 (~7GB VRAM, fits 12GB) — fastest kernels + numerically stable.
        # NOTE: full float16 causes Qwen-VL garbage output; bf16 avoids it at same speed/VRAM.
        try:
            self.model = Qwen2_5_VLForConditionalGeneration.from_pretrained(
                self.model_id,
                torch_dtype=torch.bfloat16,
                attn_implementation=attn_impl,
            ).to("cuda")
            self.processor = AutoProcessor.from_pretrained(self.model_id, min_pixels=self.min_pixels, max_pixels=self.max_pixels)
            self.connected = True
            self.worker_thread.start()
            logger.info("Local ER Brain Online (bf16): spatial reasoning on RTX 5070 Ti.")
            try:
                _free, _ = torch.cuda.mem_get_info()
                if _free < 1.5e9:
                    print(f"⚠️ LOW VRAM: only {_free/1e9:.1f}GB free after loading the pilot — CLOSE games/"
                          f"browsers! Low headroom makes inference SPILL to system RAM = 10-100x SLOWER "
                          f"(this is the 'AI never moves / 270s' trap).")
                else:
                    print(f"✅ VRAM headroom after pilot load: {_free/1e9:.1f}GB free (good — no spill).")
            except Exception:
                pass
            return True
        except Exception as e:
            logger.error(f"bf16 load failed ({e}); falling back to 4-bit NF4...")

        # Try 2: 4-bit NF4 fallback (~3.5GB VRAM, slower kernels on Blackwell)
        try:
            bnb_config = BitsAndBytesConfig(
                load_in_4bit=True,
                bnb_4bit_compute_dtype=torch.bfloat16,
                bnb_4bit_quant_type="nf4",
                bnb_4bit_use_double_quant=True,
            )
            self.model = Qwen2_5_VLForConditionalGeneration.from_pretrained(
                self.model_id,
                quantization_config=bnb_config,
                torch_dtype=torch.bfloat16,
                attn_implementation=attn_impl,
                device_map={"": 0},   # bitsandbytes: place via device_map, NOT .to()
            )
            self.processor = AutoProcessor.from_pretrained(self.model_id, min_pixels=self.min_pixels, max_pixels=self.max_pixels)
            self.connected = True
            self.worker_thread.start()
            logger.info("Local ER Brain Online (4-bit NF4 fallback).")
            return True
        except Exception as e2:
            logger.error(f"ER Brain total failure: {e2}")
            return False

    def set_director_intent(self, intent_text):
        """Called when Gemini 2.0 Flash (Cloud) sends a new master plan."""
        with self._lock:
            self.current_director_intent = intent_text
            self.director_intent_time = time.time()
            logger.info(f"Local ER received new Director Intent: {intent_text[:50]}...")

    def update_state(self, frames, sensors, detections):
        """frames: a LIST of recent FUSED frames (a short live video, sensors overlaid) — or a single
        frame (back-compat). We keep only the freshest buffer to avoid lag queueing."""
        if not self.connected:
            return
        with self._lock:
            self._input_queue = [{
                "frames": frames,
                "sensors": sensors,
                "detections": detections
            }]

    def get_latest_decision(self):
        with self._lock:
            return self._latest_decision

    def note_blocked(self, reason):
        """The director's safety layer clamped/blocked the pilot's last command — tell the pilot in its
        next prompt so it RE-ROUTES instead of repeating the blocked command (fixation breaker)."""
        self._blocked_hint = str(reason)[:160]
        self._blocked_hint_time = time.time()

    def debug_once(self, frames, sensors, detections, intent=None, max_new_tokens=200):
        """TEST HELPER (not used in flight): run ONE synchronous inference on given (fake) inputs and
        return the RAW model text + latency, so you can inspect how the pilot reasons over a scene.
        Works in either mode (server if up, else in-process)."""
        if intent:
            self.set_director_intent(intent)
        if not isinstance(frames, list):
            frames = [frames]
        sel = [self._resize_frame(fr) for fr in frames[-self._video_frames:]]
        sensor_text = self._build_sensor_text(sensors, detections)
        user_text = self._build_user_text(sensor_text)
        _saved = self.max_new_tokens
        self.max_new_tokens = max_new_tokens
        t0 = time.time()
        try:
            if self.mode == "server" and self._ping_server():
                out = self._infer_server(sel, user_text)
            else:
                _load_local_inference_libs()
                out = self._infer_local(sel, user_text)
        finally:
            self.max_new_tokens = _saved
        return out, (time.time() - t0), sensor_text
            
    def _build_sensor_text(self, sensors, detections):
        lines = []

        # Director intent + timing context
        intent_age = time.time() - self.director_intent_time if self.director_intent_time else 999
        lines.append(f"DIRECTOR'S TARGET INTENT: {self.current_director_intent}")
        lines.append(f"(Intent age: {intent_age:.1f}s ago — {'FRESH' if intent_age < 3 else 'STALE, extrapolate and continue pursuing it'})")
        lines.append("---")

        # YOUR RECENT TRAJECTORY (so you maintain smooth continuity)
        if self._decision_history:
            lines.append("YOUR LAST DECISIONS (maintain momentum):")
            for i, prev in enumerate(self._decision_history[-3:]):
                f = prev.get('flight', {})
                lines.append(f"  t-{len(self._decision_history)-i}: vx={f.get('vx',0)} vy={f.get('vy',0)} vz={f.get('vz',0)} yaw={f.get('yaw_rate',0)}")
            lines.append("---")

        # FIXATION SAFETY NET — grounding nudge, NOT a dictated maneuver. The sampling fix already breaks
        # most loops; if the model STILL repeats, we make IT re-observe and reason from the live numbers
        # (embodied chain-of-thought: "look carefully, then decide") — we do NOT tell it which way to go.
        if self._repeat_count >= 4 or self._reason_repeat >= 4:
            n = max(self._repeat_count, self._reason_repeat) + 1
            lines.append(f"⚠️ YOU ARE LOOPING: your last ~{n} decisions barely changed and made no progress — "
                         f"that means you STOPPED re-reading the sensors. Do this NOW: re-read the LiDAR/ToF "
                         f"clearances above, name in your reasoning the nearest obstacle AND the most-open "
                         f"direction, then commit to a genuinely DIFFERENT move that heads into open space. "
                         f"Do not repeat the previous command.")
        if self._blocked_hint and (time.time() - self._blocked_hint_time) < 3.0:
            lines.append(f"⚠️ BLOCKED: {self._blocked_hint}")

        # ToF is provided at the TOP level (t1-t4, or tof_front/right/back/left) — NOT nested under a
        # 'tof' key. Read it directly so the directional close-range ToF actually reaches the pilot.
        _tf = sensors.get('t1', sensors.get('tof_front'))
        _tr = sensors.get('t2', sensors.get('tof_right'))
        _tb = sensors.get('t3', sensors.get('tof_back'))
        _tl = sensors.get('t4', sensors.get('tof_left'))
        if any(v not in (None, 9999, -1) for v in (_tf, _tr, _tb, _tl)):
            lines.append(f"ToF (cm): F={_tf}, R={_tr}, B={_tb}, L={_tl}")
        if sensors.get('lidar_min_dist'):
            lines.append(f"Lidar Min Dist: {sensors['lidar_min_dist']}cm at {sensors.get('lidar_min_angle')}deg")
        if sensors.get('altitude'):
            lines.append(f"Altitude: {sensors['altitude']}m")
        if sensors.get('depth_to_subject_m'):
            lines.append(f"MiDaS Depth to Subject: {sensors['depth_to_subject_m']:.1f}m")
        if sensors.get('battery'):
            lines.append(f"Battery: {sensors['battery']}%")
        if sensors.get('speed'):
            lines.append(f"Speed: {sensors['speed']:.1f}m/s")
        if sensors.get('heading'):
            lines.append(f"Heading: {sensors['heading']:.0f}deg")

        fd = sensors.get('flight_dynamics') or {}
        if fd:
            lines.append(f"FLIGHT DYNAMICS: airborne={fd.get('airborne')} throttle={fd.get('throttle_pct')}% "
                         f"cell={fd.get('cell_voltage_v')}V climb={fd.get('climb_rate_ms')}m/s "
                         f"trim r/p={fd.get('roll_deg')}/{fd.get('pitch_deg')}deg")
        cap = sensors.get('capabilities') or {}
        if cap:
            lines.append(f"DRONE LIMITS (stay within): lean<={cap.get('max_lean_deg')}deg "
                         f"climb<={cap.get('max_climb_ms')} descent<={cap.get('max_descent_ms')} "
                         f"speed<={cap.get('max_horiz_speed_ms')}m/s vaccel<={cap.get('max_vert_accel_ms2')}")
        # LIVE WIND (FC EKF estimate) — INFO only. The FC's flight mode (GUIDED/Loiter/PosHold + GPS) does
        # the wind rejection; the pilot just picks intent. Shown so it can choose a steadier shot if gusty.
        _wspd = sensors.get('wind_speed_ms', (sensors.get('flight_dynamics') or {}).get('wind_speed_ms'))
        if _wspd is not None and float(_wspd) > 0.3:
            lines.append(f"WIND: {float(_wspd):.1f} m/s (the FC's mode holds against it — you just fly the shot).")

        if sensors.get('spatial'):
            lines.append(f"SPATIAL GRID: {sensors['spatial']}")
        if sensors.get('spatial_closest_m', 9999) < 500:
            lines.append(f"NEAREST OBSTACLE: {sensors['spatial_closest_m']:.1f}cm ({sensors.get('spatial_closest_dir', '?')})")
        # MISSION TARGET (goal layer): the thing the director told you to find is IN VIEW right now — head
        # toward it and keep it framed. yaw=face_subject centres it; head toward its bearing (unless blocked).
        _mt = sensors.get('mission_target')
        if _mt:
            _b = _mt.get('bearing_deg', 0)
            _side = "ahead" if abs(_b) < 12 else ("your RIGHT" if _b > 0 else "your LEFT")
            lines.append(f"🎯 MISSION TARGET '{_mt.get('class')}' IN VIEW at {_b:+.0f}deg ({_side})"
                         f"{', %.1fm' % _mt['distance_m'] if _mt.get('distance_m') else ''} — APPROACH it and "
                         f"FRAME it (head toward it, yaw=face_subject).")

        if detections:
            dets = []
            for d in detections[:4]:
                s = f"{d['class']} ({d.get('confidence', 0):.2f})"
                if d.get('distance_m') is not None:
                    s += f" @ {d['distance_m']}m {d.get('bearing', '')}".rstrip()
                dets.append(s)
            lines.append(f"OBJECTS (label, conf, distance): {', '.join(dets)}")

        return "\n".join(lines)

    # ── Shared frame + prompt helpers ───────────────────────────────────────────────────
    def _resize_frame(self, fr):
        """Downscale a BGR frame to the vision dim cap (keeps tokens + JPEG payload small)."""
        h, w = fr.shape[:2]
        if max(h, w) > self.max_frame_dim:
            scale = self.max_frame_dim / max(h, w)
            fr = cv2.resize(fr, (int(w * scale), int(h * scale)))
        return fr

    def _system_text(self):
        """The STATIC pilot briefing — identical every tick, so it goes in a SYSTEM message that sits
        BEFORE the (changing) frames. That makes it a stable token PREFIX → vLLM prefix-caches it → it
        prefills ~once instead of re-processing ~1500 tokens every frame. Measured 3.2x latency win
        (2233ms→699ms @5 frames): the big prompt must NEVER sit after the images or caching is defeated."""
        _cine = CINEMATIC_FRAMING_PROMPT if self.cinematic_mode else ""
        _hybrid_override = (
            "\n\n### OUTPUT OVERRIDE — HYBRID SPHERICAL CONTROL (THIS SUPERSEDES ANY OUTPUT FORMAT ABOVE):\n"
            "Do NOT output velocity numbers (you are unreliable at the exact m/s). Instead pick the "
            "3D MOVEMENT INTENT — direction + vertical slope + pace — as compact one-line JSON:\n"
            '{"head":"fwd|fwd_left|left|back_left|back|back_right|right|fwd_right|hover"}\n'
            "That single field is USUALLY all you output (keep it SHORT = fast). head = the direction that "
            "makes safe progress toward the goal (into the MOST OPEN space; front open => fwd; turn only "
            "when the front is blocked; hover only if boxed in). ONLY when you need something beyond level "
            "cruising, ADD the relevant extra field(s): \"climb\":\"climb|dive|climb_steep|dive_steep\" (a "
            "vertical slope), \"pace\":\"creep|fast\" (slow for precision / fast in the open), "
            "\"yaw\":\"face_subject|scan_left|scan_right\" (face_subject keeps the tracked subject centred — "
            "combine with a sideways head to ORBIT it). Omitted fields default to level/cruise/face_travel. "
            "The flight computer converts your intent into the exact clearance-safe 3D velocity+yaw — you "
            "ONLY choose the intent; cinematic moves EMERGE, none are pre-programmed. Output ONLY the JSON, "
            "as SHORT as possible."
        ) if self._hybrid else ""
        return (
            f"{ER_BRAIN_PROMPT}{_cine}\n\n"
            f"The image(s) in the next message are CONSECUTIVE LIVE VIDEO FRAMES (oldest→newest) from the "
            f"drone's camera. The SENSOR FEED is OVERLAID on them: green object boxes with per-object "
            f"distance, ToF at the edges (F/B/L/R cm), a DEPTH map inset (top-right), a 360° LiDAR top-down "
            f"map inset (top-left), and a NEAREST-obstacle banner. Use the MOTION across the frames PLUS the "
            f"overlaid sensors for full 3D environmental understanding.{_hybrid_override}"
        )

    def _build_user_text(self, sensor_text):
        """The DYNAMIC per-frame text (small): only the live numeric sensor feed + the decide cue. The
        static briefing lives in _system_text() so it can be cached. Keep this short so each tick's
        uncacheable prefill (frames + this) stays cheap."""
        return f"SENSOR DATA (numeric):\n{sensor_text}\n\nDecide NOW. Output ONLY the tiny one-line JSON."

    def _encode_jpeg_b64(self, fr):
        """BGR frame -> base64 JPEG data URL for the OpenAI-style image_url content."""
        ok, buf = cv2.imencode(".jpg", fr, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality])
        if not ok:
            return None
        return "data:image/jpeg;base64," + base64.b64encode(buf.tobytes()).decode("ascii")

    def _infer_server(self, sel_frames, user_text):
        """FAST PATH: POST the frames + prompt to the vLLM server, return the model's text.
        MESSAGE ORDER IS PERFORMANCE-CRITICAL: static briefing as a SYSTEM message FIRST (cacheable
        prefix), then the USER message = [frames] + tiny dynamic sensor text. Never glue the big prompt
        after the frames (defeats prefix caching → ~3x slower)."""
        content = []
        for fr in sel_frames:
            url = self._encode_jpeg_b64(fr)
            if url:
                content.append({"type": "image_url", "image_url": {"url": url}})
        content.append({"type": "text", "text": user_text})
        payload = {
            "model": self._served_name,
            "messages": [
                {"role": "system", "content": self._system_text()},
                {"role": "user", "content": content},
            ],
            "max_tokens": self.max_new_tokens,
            # ANTI-DEGENERATION SAMPLING (always on): a small temperature + nucleus top_p + repetition/
            # frequency/presence penalties stop the greedy self-reinforcing repeat loop at the decoder.
            # The fixation detector escalates temperature/penalties further while stuck.
            "temperature": self._temperature,
            "top_p": self._top_p,
            "repetition_penalty": self._repetition_penalty,
            "frequency_penalty": self._frequency_penalty,
            "presence_penalty": self._presence_penalty,
        }
        # Force a valid JSON object (no ``` fences, no prose) → always parseable, ~free. NOTE: tried a
        # strict json_schema + xgrammar disable_any_whitespace to also strip spaces (fewer tokens) — on
        # this vLLM 0.23 build it didn't strip whitespace and forced a slower backend, so we use the loose
        # json_object (auto backend): the model is compact most calls, the robust parser handles the rest.
        if self._json_mode:
            payload["response_format"] = {"type": "json_object"}
        body = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(
            f"{self.server_url}/v1/chat/completions", data=body,
            headers={"Content-Type": "application/json"}, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=self._http_timeout) as r:
                resp = json.loads(r.read().decode("utf-8"))
            return resp["choices"][0]["message"]["content"]
        except urllib.error.URLError as e:
            logger.error(f"vLLM server request failed ({e}); is start_vllm_pilot.sh running?")
            return None

    def _infer_local(self, sel_frames, user_text):
        """SLOW FALLBACK: in-process transformers generate (only if the server is down)."""
        from PIL import Image
        pil_imgs = [Image.fromarray(cv2.cvtColor(fr, cv2.COLOR_BGR2RGB)) for fr in sel_frames]
        content = [{"type": "image", "image": im} for im in pil_imgs]
        content.append({"type": "text", "text": user_text})
        # Same system-first ordering as the server path (static briefing before the frames).
        messages = [{"role": "system", "content": [{"type": "text", "text": self._system_text()}]},
                    {"role": "user", "content": content}]
        text = self.processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
        image_inputs, video_inputs = process_vision_info(messages)
        inputs = self.processor(text=[text], images=image_inputs, videos=video_inputs,
                                padding=True, return_tensors="pt").to("cuda")
        with torch.inference_mode():
            # Same anti-degeneration recipe as the server path: sample (never greedy) with top_p +
            # repetition_penalty + no_repeat_ngram_size so it can't fall into the self-reinforcing loop.
            # (HF generate has no frequency/presence penalty; repetition_penalty + n-gram block cover it.)
            _t = max(0.1, float(getattr(self, '_temperature', self._base_temperature) or self._base_temperature))
            generated_ids = self.model.generate(**inputs, max_new_tokens=self.max_new_tokens,
                                                do_sample=True, temperature=_t, top_p=self._top_p,
                                                repetition_penalty=self._repetition_penalty,
                                                no_repeat_ngram_size=3,
                                                use_cache=True)
        trimmed = [o[len(i):] for i, o in zip(inputs.input_ids, generated_ids)]
        return self.processor.batch_decode(trimmed, skip_special_tokens=True,
                                           clean_up_tokenization_spaces=False)[0]

    def _er_loop(self):
        """Continuous background loop running VLM inference."""
        while self.connected:
            time.sleep(0.01)
            
            with self._lock:
                if not self._input_queue:
                    continue
                
                time_since_last = time.time() - self.last_api_call_time
                if time_since_last < self.min_interval:
                    continue
                    
                data = self._input_queue[-1]
                self._input_queue.clear()
                intent = self.current_director_intent
                
            try:
                frames = data["frames"]
                if not isinstance(frames, list):
                    frames = [frames] if frames is not None else []
                frames = [f for f in frames if f is not None]
                if not frames:
                    continue

                # Subsample the rolling buffer to N CONSECUTIVE frames spanning the recent window
                # (a live video, newest LAST). Frames are cheap; this gives MOTION understanding.
                n = min(self._video_frames, len(frames))
                if n <= 1:
                    sel = [frames[-1]]
                else:
                    idx = sorted(set(int(round(i * (len(frames) - 1) / (n - 1))) for i in range(n)))
                    sel = [frames[i] for i in idx]
                sel = [self._resize_frame(fr) for fr in sel]

                sensor_text = self._build_sensor_text(data["sensors"], data["detections"])
                user_text = self._build_user_text(sensor_text)

                self.last_api_call_time = time.time()
                t_infer = time.time()
                if self.mode == "server":
                    output_text = self._infer_server(sel, user_text)
                else:
                    output_text = self._infer_local(sel, user_text)
                self._last_infer_s = time.time() - t_infer
                if not output_text:
                    continue
                self._infer_count += 1
                if self._infer_count % 15 == 0:
                    logger.info(f"⏱️ ER inference {self._last_infer_s*1000:.0f}ms "
                                f"(~{1.0/max(self._last_infer_s,1e-3):.1f} fps), {len(sel)} frame(s), mode={self.mode}")

                # Parse JSON — ROBUST, with truncation repair (see _parse_decision). A small 3B often
                # wraps JSON in prose OR gets clipped at the token cap mid-object; naive json.loads then
                # fails → decision dropped → the drone silently hovers on a stale decision.
                decision = _parse_decision(output_text)
                # HYBRID: the VLM emitted a DIRECTION ("head") not a velocity — convert it to a
                # clearance-safe flight vector in CODE (the say-do-gap fix). director_core still reads
                # decision['flight'] unchanged. Keep the raw head/reason for logging.
                if self._hybrid and isinstance(decision, dict) and "flight" not in decision and \
                        ("head" in decision or "climb" in decision):
                    decision["flight"] = _resolve_intent(
                        decision.get("head"), decision.get("climb"), data["sensors"],
                        pace=decision.get("pace", "cruise"), yaw_mode=decision.get("yaw", "face_travel"),
                        subject_bearing=data["sensors"].get("subject_bearing_deg"))
                    decision.setdefault("reasoning",
                        f"head={decision.get('head')} climb={decision.get('climb')} "
                        f"pace={decision.get('pace')} yaw={decision.get('yaw')}")
                if not (isinstance(decision, dict) and "flight" in decision):
                    logger.warning(f"ER decision unparseable (dropped): {str(output_text)[:100]!r}")
                if isinstance(decision, dict) and "flight" in decision:
                    with self._lock:
                        # FIXATION DETECTOR (fuzzy): the 3B repeats near-identical commands for 20-50
                        # frames into a wall, hiding it under tiny jitter. So compare the current flight
                        # command against a ROLLING WINDOW of recent ones with a tolerance — jitter can't
                        # mask a real freeze. Separately, catch a REASONING RUT (same sentence recurring).
                        cur_sig = _flight_sig(decision.get('flight', {}))
                        self._repeat_count = sum(1 for s in self._recent_sigs if _sig_near(s, cur_sig))
                        self._recent_sigs.append(cur_sig)
                        if len(self._recent_sigs) > 8:
                            self._recent_sigs.pop(0)

                        cur_reason = _norm_reason(decision.get('reasoning', ''))
                        self._reason_repeat = (sum(1 for r in self._recent_reasons if r == cur_reason)
                                               if cur_reason else 0)
                        self._recent_reasons.append(cur_reason)
                        if len(self._recent_reasons) > 8:
                            self._recent_reasons.pop(0)

                        # Escalate the SAMPLING KNOBS the longer we stay stuck (temperature + repetition
                        # penalty) so the decoder is pushed harder out of any residual loop. Base sampling
                        # already prevents most degeneration; this is the safety net for a stubborn rut.
                        # HYBRID: repeating the SAME safe direction (e.g. 'fwd' down an open corridor) is
                        # CORRECT, not a rut — the code owns the velocity and re-reads clearance every tick,
                        # so a real block still forces a different head. So DON'T escalate/​nudge in hybrid.
                        if self._hybrid:
                            self._repeat_count = self._reason_repeat = 0
                            self._temperature = self._base_temperature
                            self._repetition_penalty = float(os.getenv("PILOT_REP_PEN", "1.15"))
                        else:
                            stuck = max(self._repeat_count, self._reason_repeat)
                            if stuck >= 8:
                                self._temperature = max(self._base_temperature, 0.9)
                                self._repetition_penalty = 1.3
                            elif stuck >= 4:
                                self._temperature = max(self._base_temperature, 0.7)
                                self._repetition_penalty = 1.2
                            else:
                                self._temperature = self._base_temperature
                                self._repetition_penalty = float(os.getenv("PILOT_REP_PEN", "1.15"))
                        self._latest_decision = decision
                        self._decision_history.append(decision)
                        if len(self._decision_history) > self._max_history:
                            self._decision_history.pop(0)
                else:
                    logger.error(f"ER Brain output unparseable/no flight: {output_text[:120]}...")
                    
            except Exception as e:
                logger.error(f"ER Brain inference error: {e}")
                time.sleep(2)
