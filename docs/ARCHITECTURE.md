# Architecture

Technical design of the Autonomous Cinematic Drone control system.

---

## 1. Design principles

Four principles govern the system. Each was adopted for a specific engineering reason and none has been relaxed.

**1.1 — Semantics from the model, geometry from code.**
Vision-language models are reliable at categorical spatial judgement and unreliable at continuous numeric output. The architecture places the boundary exactly there. See §3.

**1.2 — No hardcoded maneuvers.**
There is no `orbit()`, `follow()` or `approach()` function. Cinematic behaviour emerges from composing per-tick intent. A hardcoded maneuver library would make the system a macro player rather than an autonomous agent.

**1.3 — The AI does not duplicate the flight controller.**
Stabilisation, wind rejection, position hold and low-level avoidance are ArduPilot's responsibilities and are handled there. The AI selects intent and flight *mode*. This keeps the aircraft controllable if the AI stops, and avoids two controllers fighting the same disturbance.

**1.4 — Safety is deterministic.**
Every safety-critical decision — clearance caps, direction correction, envelope clamping, return policy — is executed by auditable code, never by the model. The model's directional choice is treated as a *preference* that code may override.

---

## 2. System topology

```
┌──────────────────────── GROUND STATION (laptop, RTX GPU) ────────────────────────┐
│                                                                                   │
│   Gemini 2.5 Flash ──── strategic intent (one-shot per objective)                 │
│           │                                                                       │
│           ▼                                                                       │
│   ┌───────────────┐     ┌──────────────────────┐     ┌────────────────────────┐   │
│   │ Perception    │────▶│ 2.5D Spatial Grid    │────▶│ Qwen2.5-VL-7B (vLLM)   │   │
│   │ YOLO-World    │     │ LiDAR+ToF+depth      │     │ categorical intent      │   │
│   │ Depth-Any-V2  │     │ obstacle memory      │     └───────────┬────────────┘   │
│   │ (threaded)    │     └──────────────────────┘                 │                │
│   └───────────────┘                                              ▼                │
│                                                    ┌──────────────────────────┐   │
│                                                    │ Intent Resolver          │   │
│                                                    │ spherical geometry       │   │
│                                                    │ clearance + envelope     │   │
│                                                    │ direction correction     │   │
│                                                    └───────────┬──────────────┘   │
└────────────────────────────────────────────────────────────────┼──────────────────┘
                                          WebSocket (Tailscale)  │
┌─────────────────────── AIRCRAFT ───────────────────────────────┼──────────────────┐
│                                                                ▼                  │
│   YDLIDAR X2 ──┐                                  ┌──────────────────────────┐    │
│   ESP32 ToF  ──┼─────────────────────────────────▶│ Companion Bridge         │    │
│   GoPro/RTSP ──┘                                  │ MAVLink translation       │    │
│                                                   │ mission upload            │    │
│                                                   │ LiDAR→OBSTACLE_DISTANCE   │    │
│                                                   │ SLAM→VISION_POSITION_EST  │    │
│                                                   └───────────┬──────────────┘    │
│                                                               │ MAVLink @57600    │
│                                                               ▼                   │
│                                              ┌────────────────────────────────┐   │
│                                              │ ArduPilot FC (custom build)    │   │
│                                              │ stabilise · wind · AVOID · OA  │   │
│                                              └────────────────────────────────┘   │
└───────────────────────────────────────────────────────────────────────────────────┘
```

---

## 3. The hybrid intent resolver

### 3.1 Problem statement

Direct velocity output from the vision-language model produced **5% correct navigation decisions** despite **98% correct directional reasoning**. The model would correctly identify the open sector in prose and then emit a velocity vector pointing elsewhere. This is a grounding failure between categorical understanding and continuous output.

### 3.2 Intent schema

The model emits a compact JSON object. Only `head` is mandatory; other fields are supplied when non-default, which keeps output near six tokens.

| Field | Domain | Meaning |
|---|---|---|
| `head` | `fwd`, `fwd_left`, `left`, `back_left`, `back`, `back_right`, `right`, `fwd_right`, `hover` | Body-frame travel azimuth |
| `climb` | `dive_steep`, `dive`, `level`, `climb`, `climb_steep` | Flight-path elevation slope |
| `pace` | `creep`, `slow`, `cruise`, `fast` | Speed qualifier |
| `yaw` | `face_travel`, `face_subject`, `scan_left`, `scan_right`, `hold` | Yaw *behaviour*, never a rate |

### 3.3 Geometric resolution

Azimuth θ and elevation φ derive from the categorical tokens. Speed *s* is the minimum of the clearance-derived stopping-distance cap (scaled by pace) and the aircraft's live maximum horizontal speed read from flight-controller parameters:

```
vx = s · cos φ · cos θ        (forward, +)
vy = s · cos φ · sin θ        (right, +)
vz = s · sin φ                (up, +)   clamped to PILOT_SPEED_UP / _DN
```

Yaw rate is computed from the yaw *behaviour* — `face_subject` drives the tracked subject bearing toward zero, `face_travel` aligns with the travel vector — always clamped to the aircraft's maximum yaw rate.

**Stopping-distance speed cap** (measured clearance → maximum speed):

| Clearance | Max speed |
|---|---|
| < 60 cm | 0.00 m/s |
| 60–100 cm | 0.20 m/s |
| 100–150 cm | 0.35 m/s |
| 150–250 cm | 0.50 m/s |
| > 250 cm | 0.60 m/s |

### 3.4 Direction safety correction

The model's azimuth is a preference. If the corresponding sector measures under 80 cm, code re-steers to the **most forward open sector** — sorted by absolute azimuth first, then by clearance — so the aircraft turns and makes progress rather than reflexively reversing. Only when every sector is blocked does it hover.

### 3.5 Emergent maneuvers

No maneuver is scripted. Composition produces them:

| Sustained intent | Emergent behaviour |
|---|---|
| `head: right` + `yaw: face_subject` | Orbit around the subject |
| `head: fwd` + `yaw: face_subject` | Approach while framing |
| `head: fwd` + `climb: climb` + `yaw: face_subject` | Rising reveal |
| `yaw: scan_right` + `head: hover` | Stationary search sweep |

---

## 4. Perception and sensor fusion

### 4.1 Sensor complement

| Sensor | Coverage | Contribution |
|---|---|---|
| YDLIDAR X2 | 360° planar, 8 m | Primary obstacle ring; SLAM input |
| 4× VL53L1X | 45° outboard, 2 up / 2 down | Vertical blind-spot coverage |
| Monocular metric depth | ~90° forward cone, dense | Volumetric fill, sub-LiDAR-plane obstacles |
| YOLO-World | Camera FOV | Open-vocabulary semantic detection |

The LiDAR is planar — it cannot see a tree canopy or a low table. The angled ToF pair and the camera depth cone exist specifically to cover that gap.

### 4.2 Fusion grid

A 2.5D occupancy grid (20 cm cells, 10 m extent, aircraft-centred) fuses all sources. Clearances are computed from **raw sensor values in centimetres**, not from quantised cells, preserving precision.

**Short-term obstacle memory.** The depth cone covers roughly 90°; without memory, obstacles vanished from the grid the moment the aircraft or gimbal turned away. Camera-derived points persist for 3 seconds, ego-compensated for forward motion and yaw, and are discarded immediately when the camera re-observes their sector — fresh observation always wins.

### 4.3 Gimbal-aware depth projection

The camera is gimbal-mounted, so depth bearings must be offset by the gimbal's pan angle before fusion. Without this correction, a panned camera writes obstacles into the wrong body-frame sector. Frames with steep tilt (> 35°) are excluded from horizontal-obstacle fusion, as they observe floor or ceiling rather than the navigation plane.

### 4.4 Motion-triangulation depth anchoring

Monocular metric depth carries 2–10% scale error. Because the aircraft is always moving, parallax is available at no cost: sparse features are tracked between frames, rotational flow is removed using flight-controller attitude deltas, and depth is triangulated from translational flow against the known baseline:

```
Z = (a · f_t) / (f_t · f_t),    a = (u·t_z − f·t_x,  v·t_z − f·t_y)
```

The median ratio of triangulated to model depth becomes a robust scale correction. Error scales as `σ_Z ≈ Z²·σ_px / (f·B)` — approximately 1.5 cm at 2 m with a 30 cm baseline. The estimator returns no correction unless baseline, parallax and inlier count all pass thresholds, falling back to the time-of-flight anchor.

---

## 5. Control paths

The bridge selects the delivery mechanism based on position availability. Both paths are verified against real firmware.

### 5.1 Outdoor — GUIDED velocity setpoints

With a 3D fix, velocity is delivered as `SET_POSITION_TARGET_LOCAL_NED`. ArduPilot's position controller executes it, rejecting wind and holding the commanded velocity.

### 5.2 Indoor — ALT_HOLD attitude override

Without a position estimate, ArduPilot ignores GUIDED velocity setpoints. Velocity is converted to attitude commands via RC override in ALT_HOLD, where the barometer holds altitude:

```
rc1 (roll)     = 1500 + vy · 250
rc2 (pitch)    = 1500 − vx · 250
rc3 (throttle) = 1500 − vz · 250
rc4 (yaw)      = 1500 + yaw_rate · 250        all clipped to [1100, 1900]
```

⚠️ ArduPilot invalidates RC overrides after 3 seconds. The ground station streams at ~15 Hz, well inside that window, and **the expiry is a deliberate failsafe**: if the AI stalls, the sticks neutralise rather than holding a stale command. It must not be extended.

### 5.3 Obstacle data to the flight controller

The full LiDAR ring is published as `OBSTACLE_DISTANCE` — 72 sectors of 5°, in body-FRD frame. This enables ArduPilot's native proximity avoidance and BendyRuler/Dijkstra path deviation during autonomous missions.

An earlier implementation sent only the single nearest return, always labelled *forward*. A wall to the left was reported as an obstacle ahead, so the flight controller braked the wrong axis. The full ring corrects this. Requires `PRX1_TYPE=2` (MAVLink proximity).

### 5.4 Indoor position estimate

The SLAM module publishes `VISION_POSITION_ESTIMATE` when no GPS fix is present, giving the EKF an external position source. EKF source parameters are **not** auto-configured, as switching them automatically would break outdoor GPS flight.

---

## 6. Safety architecture

Five independent layers, ordered from model to motors. Each is active in normal flight.

| Layer | Location | Function |
|---|---|---|
| 1 — Clearance speed cap | Resolver | Stopping-distance-limited speed from measured clearance |
| 2 — Direction correction | Resolver | Re-steers when the model's chosen sector is blocked |
| 3 — Envelope clamp | Resolver | Limits to live FC parameters (lean, climb, descent, yaw, speed) |
| 4 — Reactive avoidance | Director | Brakes the component entering a near sector; adds repulsion |
| 5 — FC-native AVOID / OA | Flight controller | Proximity braking and path deviation, **operates without the AI** |

**Return policy.** Destination (home / operator / land-in-place) and battery threshold are operator-defined in the application. Without a position fix, home and operator returns automatically degrade to land-in-place rather than attempting a blind return.

**Landing protection.** Landing commands set a command lockout that suppresses the AI velocity stream for 16 seconds, plus a disarm backstop that guarantees motor stop if the landing detector fails.

---

## 7. Two-brain division

| | Strategic (Gemini) | Pilot (Qwen-VL) |
|---|---|---|
| Invocation | Once per objective | Every control tick |
| Latency budget | Seconds | ~250 ms |
| Location | Cloud | Local |
| Output | Mission intent, subject choice, shot concept | Categorical navigation intent |
| Failure mode | Mission proceeds on last known intent | Safe default; deterministic safety layers remain active |

Network failure cannot stall the control loop, because no cloud call sits inside it.

---

## 8. Performance

| Stage | Cost |
|---|---|
| YOLO-World (TensorRT FP16) | 11.5 ms |
| Metric depth | ~40 ms |
| Pilot inference (Qwen-7B-AWQ, vLLM) | ~209 ms |
| Resolver + safety | < 1 ms |
| Bridge → MAVLink | < 1 ms |
| **End-to-end loop** | **236 ms median (4.2 Hz)** |

Perception runs on a background thread, so loop time is `max(perception, inference)` rather than their sum. Latency reduction came from compact categorical output (~85% fewer output tokens), a persistent vLLM server with CUDA graph capture, prefix caching of the static prompt, and TensorRT export of the detectors.

---

## 9. Known limitations

| Limitation | Impact | Mitigation path |
|---|---|---|
| No powered-flight validation | All results are simulation or firmware-in-the-loop | Staged flight-test programme |
| Planar LiDAR | Blind to canopy and sub-plane obstacles | Angled ToF pair; camera depth cone; panoramic camera planned |
| Ground-station dependency | Control loop requires the link | Onboard inference (Jetson-class) planned |
| Metric depth tuned indoors | Degraded accuracy outdoors | Triangulation anchoring reduces sensitivity |
| Local path planner opt-in | A* costmap disabled by default, frame conventions untested | Enable after simulation validation |
| Indoor position not hardware-validated | SLAM verified in simulation only | Requires EKF configuration and bench test |
