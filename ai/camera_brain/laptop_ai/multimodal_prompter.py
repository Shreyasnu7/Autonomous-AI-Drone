# laptop_ai/multimodal_prompter.py
import json
import base64
import os
import aiohttp
import time
import asyncio
from laptop_ai.config import OPENAI_API_KEY, OPENAI_MODEL, DEEPSEEK_API_KEY, DEEPSEEK_URL, USE_LOCAL_LLM

# Import the User's Advanced Reasoning Engine
try:
    from ai.shot_intent.reasoning.intent_reasoner import ShotIntentReasoner
except ImportError:
    print("⚠️ Warning: ShotIntentReasoner not found. Using fallback.")
    ShotIntentReasoner = None

SYSTEM_PROMPT = """
You are the MASTER CINEMATIC DIRECTOR + PILOT of an autonomous AI drone — a world-class
film director, cinematographer, AND flight controller fused into one mind.

CORE PRINCIPLES (NON-NEGOTIABLE):
- NEVER output a simplified or generic command. Even a trivial request ("move up 15cm") must
  account for EVERYTHING: every obstacle distance, depth, altitude, battery, and what is in frame.
- You generate behavior yourself. You are NOT limited to presets — design the exact motion.
- A complex request (e.g. "film a sports-car commercial", "orbit the subject while ascending and
  pull focus") is a MULTI-PHASE sequence executed in order, not one action.
- SAFETY: read the sensor block. If an obstacle is closer than your intended path, route AROUND it.
  The lidar/ToF distances are TRUTH (a net 90cm ahead is real even if the camera sees past it).

PHYSICAL PLATFORM — you fly a SPECIFIC airframe; account for its DYNAMICS in EVERY plan using the LIVE
flight-controller readings in the `flight_dynamics` block. NEVER assume thrust/hover numbers — READ them:
- Fixed geometry only: DJI F450 quad-X, diagonal 450mm, arm ≈ 0.225m. All-up weight ≈ 1.6kg for THIS
  build — treat this as a rough PRIOR for sanity, not a control value.
- READ flight_dynamics every plan: armed, airborne, mode, pack_voltage_v, cell_voltage_v, throttle_pct,
  climb_rate_ms, roll_deg/pitch_deg, altitude_agl_m. Derive the HOVER point from the LIVE throttle while
  hovering, thrust HEADROOM from voltage + throttle, and current TRIM from the live roll/pitch. These
  measured values are the TRUTH; your weight prior is only a cross-check.
- Power & lift: climb needs thrust > weight, descend < weight, hover = weight. As pack/cell VOLTAGE sags
  under load, available thrust drops — when voltage is low OR throttle is already high, REDUCE speed/
  aggressiveness and avoid steep climbs. Use climb_rate_ms to confirm the FC is achieving your move.
- Center of mass: near centre, slightly LOW (battery underslung), biased toward the GoPro/gimbal — read
  the live roll/pitch to see the REAL trim; keep maneuvers within the small angle the FC holds stably.
- Stabilization: ArduPilot self-levels & holds attitude (STABILIZE/ALT_HOLD). Never command velocities or
  implied attitudes beyond what the live throttle/voltage headroom can support.
- MOTION ENVELOPE is PHASE-DEPENDENT. Gentle, low-jerk ease-in/ease-out is COMPULSORY during TAKEOFF and
  LANDING (safety-critical). IN FLIGHT you are NOT capped at "gentle" — fly to the drone's REAL limits in
  flight_dynamics.capabilities (max_lean_deg, max_climb_ms, max_descent_ms, max_horiz_speed_ms,
  max_vert_accel_ms2, max_horiz_accel_ms2) when the shot demands it. Those numbers are read LIVE from the
  FC — they are the TRUE envelope: never exceed them, but don't artificially throttle below them in flight.
- ALWAYS ramp velocity (ease-in/out) and budget an accel+decel phase inside the distance so moves END on
  target without overshoot — momentum is real. MATCH aggressiveness to the moment: a calm cinematic glide
  uses a fraction of the envelope; a fast reposition can use the full FC limits. (Still derate when
  flight_dynamics shows low voltage / high throttle — thrust headroom shrinks.)
- Clearance scales with speed: keep every rotor tip ≥ 2×arm (0.45m) from anything; faster motion needs
  MORE standoff because stopping distance grows with v². Slow before tight gaps.

ACTIVE VERTICAL PERCEPTION — the YDLidar is a 360° HORIZONTAL plane only: it sees all around at ONE
height but is BLIND above and below that plane. So actively SAMPLE the vertical dimension:
- ONLY when AIRBORNE (flight_dynamics.airborne == true). ON THE GROUND the drone CANNOT tilt or bob to
  sample vertically — if it is grounded (airborne == false) and you need vertical awareness, FIRST take
  off to a safe low hover, THEN sample. Never plan a tilt/bob while grounded.
- Once airborne, insert small vertical bobs (±0.1-0.2m up/down) and brief directional nudges that
  momentarily tilt the airframe, so the horizontal scan plane sweeps through different heights/angles and
  reveals obstacles above/below the plane (low furniture, table edges, ceilings, shelves, hanging wires).
- Do this when entering a new area, BEFORE any climb/descent, or whenever vertical clearance is uncertain
  — especially in BLIND MODE (no camera). Fuse the bobbed scans + ToF + monocular depth + altitude into
  your 3D mental model before committing to the main move.
- BOUND it by hardware + environment: bob only within safe altitude (don't rise into a ceiling or sink to
  the floor — use altitude + up/forward clearance), keep tilts small enough that the FC stays level and the
  motion stays smooth, and respect battery. A gentle reconnaissance wobble, never a lurch.

INPUT you receive: the user's request, live vision (detections/layout), ALL sensors
(lidar obstacles by direction in cm, ToF, monocular depth, altitude, speed, battery), and memory.

OUTPUT — return ONE valid JSON object, nothing else:
{
  "thought_process": "Director's reasoning: the shot, why, HOW you account for each obstacle/sensor, AND the physical dynamics (mass/thrust/CoM/momentum/battery) + any vertical-perception bob you use.",
  "cinematic_style": "Visual style name (e.g. 'Teal & Orange', 'Noir', 'Golden Hour').",
  "technical_config": { "fps": 24|30|60, "shutter_angle": 180, "look": "Daylight"|"Tungsten"|"Auto" },
  "execution_plan": {
      "action": "FLY_TRAJECTORY | FOLLOW | ORBIT | EXPLORE | PATROL | FLY_THROUGH | TRACK_PATH | HOVER",
      "mission": "THE KEY FIELD. ONE clear, plain-language strategic goal the ONBOARD PILOT (a live
          vision+sensor loop) will fly CONTINUOUSLY until done — do NOT bake coordinates. Describe the
          INTENT + constraints, not a point list. The drone has NO GPS/odometry; it navigates from the
          live camera + ToF/LiDAR/depth in real time, so trust it to measure the actual room. Example:
          'Rise to ~0.5m, then fly one slow clockwise loop hugging the room's walls ~0.5m off them,
          keep the camera facing outward, return to where you started, then hover.'",
      "params": {
          // OPTIONAL. Only for a SHORT, precise, fully-known relative move (e.g. 'dolly forward 1m').
          // points = CUMULATIVE [right(+x),forward(+y),up(+z)] metres from [0,0,0]. For room-scale or
          // exploratory tasks LEAVE points EMPTY and rely on "mission" — the pilot flies it live.
          "points": [], "speed_ms": 0.2-2.0, "height_m": 0.1-3.0, "radius_m": float,
          "aggressiveness": 0.0-1.0, "yaw_rate_dps": float
      },
      "gimbal": { "pitch": -90..20, "yaw": float },
      "recording": true|false,
      "focus": "subject"|"infinity"|"manual"
  },
  "sequence_plan": {
      "full_sequence_summary": "One line describing the whole multi-phase shot (omit for single moves).",
      "phases": [ { "phase": 1, "goal": "...", "action": "...", "params": {...}, "duration_s": float } ],
      "current_phase": 1
  },
  "lighting_analysis": "Brief lighting read.",
  "hybrid_plan": [
      // PREFERRED for precise execution: decompose the mission into ordered MANEUVER steps the
      // deterministic flight code executes EXACTLY (it computes precise velocity from live sensors).
      // Each step: {"name","goal","maneuver","done":{"type","value"}, + maneuver params}
      //   maneuver ∈ ASCEND | DESCEND | GOTO | SCAN | ORBIT | FOLLOW | WALL_FOLLOW | HOLD
      //   params: target_alt_m, bearing_deg, target_label (the object to aim at, grounded by vision),
      //           target_dist_m (orbit radius / follow standoff), side(L/R), yaw_dir(L/R), pace(slow/norm)
      //   done.type ∈ alt_at | alt_below | front_within | corners | orbit | target_within | found | timeout | never
      // Use MANEUVERS, never fake coordinates: "go around the room's edges" => one
      //   {"maneuver":"WALL_FOLLOW","side":"R","done":{"type":"corners","value":4}} step.
      // Example "circle the plant then land": [
      //  {"name":"ASCEND","maneuver":"ASCEND","target_alt_m":0.6,"done":{"type":"alt_at","value":0.55}},
      //  {"name":"FIND","maneuver":"SCAN","yaw_dir":"L","done":{"type":"found"}},
      //  {"name":"APPROACH","maneuver":"GOTO","target_label":"plant","done":{"type":"target_within","value":1.0}},
      //  {"name":"ORBIT","maneuver":"ORBIT","target_label":"plant","target_dist_m":1.0,"done":{"type":"orbit","value":330}},
      //  {"name":"LAND","maneuver":"DESCEND","target_alt_m":0.0,"done":{"type":"alt_below","value":0.12}} ]
      // Leave EMPTY [] only for truly vague/freeform tasks (then the live pilot flies "mission" instead).
  ]
}

CRITICAL: You give ONE comprehensive strategic plan; the ONBOARD PILOT then flies it CONTINUOUSLY with
live vision + sensors until the goal is met. So put the real plan in "mission" as plain intent — do NOT
decompose a simple request ("go around the room's edges") into a list of made-up coordinates/corners;
the pilot measures the real room itself. Reserve "points" for short, precise, fully-known relative moves
only (and leave it empty otherwise). ALWAYS fill thought_process with how the sensors AND the physical
dynamics shaped the plan. BE BOLD and exact, but never exceed the airframe's stable, battery-supported
envelope.
"""



async def ask_gpt(user_text, vision_context=None, images=None, video_link=None, memory=None, timeout=15, sensor_data=None, api_keys={}):
    """
    Sends a multimodal prompt using the ADVANCED SHOT INTENT REASONER if available.
    Supports DeepSeek for ultra-low latency if configured.
    """
    
    # 1. Gemini-primary planner. This does NOT require ShotIntentReasoner (it calls Gemini
    # directly). Previously gated on `if ShotIntentReasoner:` — when that optional module was
    # absent (the common case), ask_gpt skipped Gemini entirely and fell to the OpenAI-only
    # legacy path below (401, no key). Always run the Gemini path.
    if True:
         class AsyncAdapterLLM:
             async def chat_async(self, system, user):
                # PRIMARY: Gemini — rotate across a POOL of keys so heavy testing survives one key's
                # free-tier quota (429/limit:0). Pool = job key + GEMINI_API_KEYS (comma-separated) +
                # GEMINI_API_KEY. First working key+model wins; exhausted keys are skipped.
                _gem_keys = []
                for _src in (api_keys.get("gemini"), os.getenv("GEMINI_API_KEYS"), os.getenv("GEMINI_API_KEY")):
                    if _src:
                        for _k in str(_src).split(","):
                            _k = _k.strip()
                            if _k and _k not in _gem_keys:
                                _gem_keys.append(_k)
                if _gem_keys:
                    try:
                        from google import genai as google_genai
                        # Attach the LIVE camera frame(s) so the AI SEES the scene, fused with the
                        # sensor/depth text below. `images` (closure) = list of JPEG bytes; empty -> sensor-only.
                        # Fully defensive: any SDK/encoding issue just drops to text+sensors (never breaks Gemini).
                        _img_parts = []
                        try:
                            from google.genai import types as _genai_types
                            for _im in (images or []):
                                if isinstance(_im, (bytes, bytearray)):
                                    _img_parts.append(_genai_types.Part.from_bytes(data=bytes(_im), mime_type='image/jpeg'))
                        except Exception as _pe:
                            print(f"⚠️ image attach skipped ({_pe}); planning on text + sensors only")
                            _img_parts = []
                        _has_img = bool(_img_parts)
                        prompt = (f"{system}\n\n=== LIVE INPUT (vision + ALL sensors + memory + user request) ===\n"
                                  f"{user}\n\n"
                                  + ("A LIVE camera image is ATTACHED — use it together with the depth/ToF/LiDAR data; "
                                     "per-object metric distances are in sensors.objects (label + distance_m + bearing). "
                                     if _has_img else
                                     "NO camera image this frame — plan SENSOR-ONLY from LiDAR/ToF/IMU. ")
                                  + "Return ONLY the JSON object. Account for EVERY sensor reading and obstacle.")
                        _contents = [prompt] + _img_parts
                        # Some keys have ZERO free-tier quota on a given model (429 'limit: 0').
                        # Try a chain so we land on a model this key can actually use.
                        env_model = os.getenv("GEMINI_PLANNER_MODEL")
                        # Newer free-tier keys have limit:0 on gemini-2.0-flash and 404 on the retired
                        # 1.5 models — only the 2.5 family is usable. Try those FIRST so we don't burn a
                        # request on a dead model every plan. 2.0-flash kept last for old-allocation keys.
                        candidates = ([env_model] if env_model else []) + [
                            "gemini-2.5-flash", "gemini-2.5-flash-lite",
                            "gemini-flash-latest", "gemini-2.0-flash",
                        ]
                        seen = set()
                        _model_list = [m for m in candidates if m and not (m in seen or seen.add(m))]
                        # TRANSIENT errors (429 quota, 503 overload/"high demand", UNAVAILABLE) are
                        # temporary — DON'T abandon Gemini and fall to the (unconfigured) OpenAI path.
                        # Cascade through the model chain, then re-sweep up to 3x with backoff.
                        # Transient (try next model) + key-level (rotate to next key) failures — anything
                        # in here means "skip and keep trying" rather than abandon Gemini for OpenAI.
                        _transient = ("429", "RESOURCE_EXHAUSTED", "limit: 0", "404",
                                      "503", "UNAVAILABLE", "overloaded", "high demand", "500", "INTERNAL",
                                      "API_KEY_INVALID", "API key not valid", "PERMISSION_DENIED", "INVALID_ARGUMENT")
                        # Force STRUCTURED JSON output so the planner can't return prose/markdown that
                        # fails json.loads (the "JSON Parse Error -> HOVER" bug). Gemini 2.x/1.5 support it.
                        try:
                            from google.genai import types as _gt
                            _gen_cfg = _gt.GenerateContentConfig(response_mime_type="application/json")
                        except Exception:
                            _gen_cfg = None
                        for _attempt in range(3):
                            for _ki, _key in enumerate(_gem_keys):
                                client = google_genai.Client(api_key=_key)
                                for gm in _model_list:
                                    try:
                                        resp = await client.aio.models.generate_content(model=gm, contents=_contents, config=_gen_cfg)
                                        if gm != _model_list[0] or _attempt or _ki:
                                            print(f"✅ Gemini planner: model={gm} key#{_ki+1}/{len(_gem_keys)} (sweep {_attempt+1})")
                                        return resp.text
                                    except Exception as me:
                                        s = str(me)
                                        if any(k in s for k in _transient):
                                            print(f"⚠️ Gemini key#{_ki+1} '{gm}' busy/quota — trying next…")
                                            continue
                                        raise
                                # this key is exhausted on every model -> rotate to the next key
                            if _attempt < 2:
                                print(f"⚠️ All {len(_gem_keys)} Gemini key(s) busy; retry sweep {_attempt+1}/2 in 2s…")
                                await asyncio.sleep(2)
                        print("⚠️ All Gemini keys+models exhausted/blocked; falling back to DeepSeek/OpenAI")
                    except Exception as e:
                        print(f"⚠️ Gemini planner failed ({e}); falling back to DeepSeek/OpenAI")

                # FALLBACK: DeepSeek / OpenAI (only if no Gemini key or Gemini errored)
                if USE_LOCAL_LLM:
                    api_key = api_keys.get("deepseek") or DEEPSEEK_API_KEY
                    url = f"{DEEPSEEK_URL}/chat/completions"
                    model = "deepseek-chat"
                else:
                    api_key = api_keys.get("openai") or OPENAI_API_KEY
                    url = "https://api.openai.com/v1/chat/completions"
                    model = OPENAI_MODEL

                payload = {
                    "model": model,
                    "messages": [{"role":"system","content":system}, {"role":"user","content":user}],
                    "max_tokens": 1000,
                    "temperature": 0.2
                }
                headers = {"Authorization": f"Bearer {api_key}", "Content-Type": "application/json"}
                async with aiohttp.ClientSession() as sess:
                    async with sess.post(url, json=payload, headers=headers, timeout=timeout) as resp:
                        if resp.status != 200: return None
                        data = await resp.json()
                        return data['choices'][0]['message']['content']

         # Instantiate Reasoning Engine
         adapter = AsyncAdapterLLM()
         # Temporary prompt path - in real app, ensure this file exists or use string
         # We will bypass the file read in Reasoner by using internal logic

         
         # Logic:
         # 1. Construct payload as Reasoner would.
         # 2. Add System Prompt.
         # 3. Send.
         
         full_context = {
             "vision": vision_context,
             "sensors": sensor_data, # INJECTED REAL SENSORS
             "memory": memory,
             "user_text": user_text
         }
         
         # Use the Adapter to send
         # We are integrating the Logic of Reasoner (Context Structuring) 
         response_text = await adapter.chat_async(SYSTEM_PROMPT, json.dumps(full_context))
         
         if not response_text:
             print("⚠️ Empty AI response (all models/keys failed) — HOVER")
             return None
         try:
             return json.loads(response_text)            # JSON-mode output: usually already clean
         except Exception:
             pass
         # Robust fallback: strip ``` fences / prose, extract the {...}, drop // comments + trailing commas.
         try:
             import re as _re
             t = response_text
             if "```" in t:
                 t = t.split("```json")[-1].split("```")[0] if "```json" in t else t.split("```")[1]
             m = _re.search(r'\{.*\}', t, _re.DOTALL)
             if m:
                 cleaned = _re.sub(r'//[^\n]*', '', m.group(0))      # strip // comments
                 cleaned = _re.sub(r',\s*([}\]])', r'\1', cleaned)   # strip trailing commas
                 return json.loads(cleaned)
         except Exception:
             pass
         print(f"JSON Parse Error in AI Response — raw(220): {str(response_text)[:220]}")
         return None

    # Fallback to legacy simplistic method if import failed
    payload = {
        "model": OPENAI_MODEL,
        "messages": [
            {"role":"system", "content":SYSTEM_PROMPT},
            {"role":"user", "content": f"User request: {user_text}\n\nVisionContext: {json.dumps(vision_context or {}, default=str)}\nSensors:{json.dumps(sensor_data or {})}\nMemory:{json.dumps(memory or {})}\nImages:{images or []}\nVideo:{video_link or ''}"}
        ],
        "max_tokens": 400,
        "temperature": 0.2,
    }

    headers = {
        "Authorization": f"Bearer {OPENAI_API_KEY}",
        "Content-Type": "application/json"
    }

    try:
        async with aiohttp.ClientSession() as sess:
            async with sess.post("https://api.openai.com/v1/chat/completions", json=payload, headers=headers, timeout=timeout) as resp:
                if resp.status != 200:
                    text = await resp.text()
                    print("OpenAI error:", resp.status, text[:400])
                    return None
                data = await resp.json()
                content = data['choices'][0]['message']['content']
                # Clean markdown
                if "```json" in content:
                    content = content.split("```json")[1].split("```")[0].strip()
                elif "```" in content:
                    content = content.split("```")[1].strip()
                return json.loads(content)
    except Exception as e:
        print(f"LLM Network Error: {e}")
        return None


