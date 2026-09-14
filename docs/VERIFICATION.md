# Verification

Test methodology and results for the Autonomous AI Drone.

Because this system commands a physical aircraft, verification is layered: each layer tests something the layer below cannot, and the highest layers exercise **real flight-control firmware** rather than mocks.

---

## 1. Verification strategy

| Layer | Question answered | Mocked | Real |
|---|---|---|---|
| 1 — Static | Does every module compile and import? | — | Source |
| 2 — Command path | Does the bridge emit the correct MAVLink for each command? | Flight controller | Bridge logic |
| 3 — Wire links | Does every message shape match between sender and receiver? | Runtime | Both sides' source |
| 4 — Algorithms | Do depth anchoring and obstacle memory behave correctly? | Sensors | Algorithms |
| 5 — Firmware-in-loop | Does **real ArduPilot** respond correctly to our commands? | Physics only | **Firmware + bridge** |
| 6 — Full mission | Does the complete AI stack fly a mission end to end? | Environment | Entire stack |

A deliberate principle: **layer 5 binds production bridge methods to real firmware**, not reimplementations. Faults found there are faults that would have occurred in flight.

---

## 2. Results summary

| Suite | Checks | Result |
|---|---|---|
| Static compilation (production modules) | 14 | ✅ 14/14 |
| Command-path tests | 37 | ✅ 37/37 |
| Wire-link verification | 44 | ✅ 44/44 |
| Depth anchor + obstacle memory | 11 | ✅ 11/11 |
| **SITL firmware-in-the-loop** | 13 | ✅ **13/13** |
| **Route missions vs. firmware** | 10 | ✅ **10/10** |
| Full-stack simulated missions | 7 | ✅ 7 completed |

---

## 3. Layer 2 — Command-path tests

**Method.** Production bridge methods are bound to a stub flight controller that records every MAVLink call. Packets are sent in the exact shapes the application and ground station emit, and the resulting MAVLink traffic is asserted.

**Representative coverage**

| Behaviour | Assertion |
|---|---|
| LAND (airborne) | `MAV_CMD_NAV_LAND` + avoidance monitor + 16 s command lockout |
| LAND (on ground) | Force-disarm (magic value 21196) |
| DISARM | 5× force-disarm + 8 s lockout |
| LOITER | `set_mode` custom mode 5 |
| RTL | Mode RTL + avoidance monitor |
| RETURN_TO_USER | GUIDED + waypoint, coordinates scaled ×10⁷ |
| SET_SPEED | `WPNAV_SPEED` parameter write (2 m/s → 200 cm/s) |
| `cmd_vel`, no GPS | ALT_HOLD + RC override, exact stick values |
| `cmd_vel`, with GPS | GUIDED velocity setpoint, no RC override |
| Lockout enforcement | Velocity stream ignored after LAND |
| Blind-return prevention | No-GPS return → land in place, never RTL |
| Application string payloads | `"ARM"` → split-arm sequence |
| Unknown command | **No** mode change as a side effect |

**Exact-value example** — no-GPS conversion for `vx=0.3, vz=−0.2`:
```
pitch = 1500 − 0.3·250 = 1425   ✅
throttle = 1500 + 0.2·250 = 1550 ✅
```

---

## 4. Layer 3 — Wire-link verification

**Method.** Both sides of every interface are parsed from source and compared. This catches the failure mode where each side is internally correct but they disagree on message shape — the class of bug that silently disabled the mission feature for weeks.

**Coverage:** ground station → bridge message shapes; every relayed command confirmed present in the bridge's handler; bridge-internal routing order; application → bridge payload parsing; pilot-brain wiring.

**Real safety gate exercised.** Earlier command-path tests stubbed the safety gate as permissive, which could have masked a gate that blocked legitimate commands. This layer instantiates the **real** gate with an obstacle at 40 cm:

| Command | Expected | Result |
|---|---|---|
| LAND, RETURN_TO_USER, LOITER, ARM | Allowed (safety-critical) | ✅ |
| SET_SPEED (unrecognised) | Allowed (default-permit) | ✅ |
| TAKEOFF at 40 cm | **Blocked** | ✅ |
| TAKEOFF with clearance | Allowed | ✅ |

---

## 5. Layer 5 — Firmware-in-the-loop (SITL) 🔬

**Method.** ArduCopter SITL — the same firmware family flashed to the aircraft — runs locally. Production bridge methods are bound to that MAVLink connection. Assertions are made against the **firmware's actual reported state**: mode, armed flag, altitude, ground speed, parameter values.

**Results — 13/13**

| # | Check | Evidence |
|---|---|---|
| 1 | GPS 3D fix | `fix=6` |
| 2 | EKF origin set | Status message |
| 3 | Bridge GUIDED → firmware GUIDED | `mode=GUIDED` |
| 4 | SET_SPEED → parameter write | `WPNAV_SPEED=350` |
| 5 | Bridge ARM → firmware armed | `armed=True` |
| 6 | TAKEOFF → climb | `alt=8.9 m` |
| 7 | **`cmd_vel` 3.0 m/s → measured ground speed** | **2.98 m/s** |
| 8 | LOITER | `mode=LOITER` |
| 9 | GPS disabled | `fix<3` |
| 10 | No-GPS `cmd_vel` → ALT_HOLD | `mode=ALT_HOLD` |
| 11 | No-GPS RC override reaches firmware | `rc3=1600` |
| 12 | RTL | `mode=RTL` |
| 13 | Landing + auto-disarm | `armed=False` |

### 5.1 Faults caught before hardware 🔬

**Takeoff would have been silently refused.** `NAV_TAKEOFF` is valid only in GUIDED, but the application's split-arm sequence arms in STABILIZE. The firmware rejected it (`COMMAND_ACK result=4`). In the field this would have presented as *"the aircraft arms but will not take off"* — an expensive diagnosis. Fixed by ensuring GUIDED before commanding takeoff.

**EKF readiness signal mis-identified.** GUIDED is refused with `requires position` for several seconds *after* a GPS 3D fix, until the EKF adopts GPS for lateral position. The true readiness event is `EKF3 using GPS`, which follows `origin set`. This converts the field heuristic "wait for the green fix" into a precise, machine-checkable condition.

---

## 6. Layer 5b — Route missions vs. firmware

**Method.** The **exact packet the application emits** when the operator taps waypoints and presses START is fed through the production bridge to real firmware.

**Results — 10/10:** packet accepted → mission handshake upload → `MISSION_ACK result=0` → bridge ARM → firmware AUTO → auto-takeoff to 8.9 m → **waypoints 1–5 all reached in order**.

### 6.1 Faults caught 🔬

**Missions never reached the flight controller over the local link.** The bridge's local packet handler had no `mission` branch; mission packets were logged as *unknown type* and discarded. Upload logic existed only in a path not in use. The map feature was non-functional despite appearing to work in the application.

**No start mechanism existed.** The application's START button set a local UI label only; nothing was transmitted.

**Missions would have stalled after arming.** With default `AUTO_OPTIONS`, AUTO waits for a pilot throttle raise before the takeoff item — impossible for an application-flown aircraft. Corrected with `AUTO_OPTIONS=3`.

**Upload handshake race.** Sending `mission_clear_all` before `mission_count` produced its own `MISSION_ACK`, which the handler mistook for upload completion. The firmware then reported `Mission upload timeout` and refused AUTO with `init failed`. Resolved by relying on the single `mission_count` transaction, which replaces the existing mission implicitly.

---

## 7. Layer 6 — Full-stack simulated missions

Three environments were used: a custom raycast simulator (dark-room, angled-ToF suite), **AI2-THOR** (photorealistic interiors — real detector performance on real imagery), and **AirSim** (outdoor flight dynamics, wind API, GPU rendering).

| # | Mission | Environment | Result |
|---|---|---|---|
| 1 | Dark-room search, observation, hazard-avoiding landing | Raycast | ✅ 56 s |
| 2 | Photorealistic target search and observation | AI2-THOR | ✅ 38 s |
| 3 | **Full autonomy** — AI chooses subject *and* landing site | AI2-THOR | ✅ 46 s |
| 4 | 8-stage indoor stress: moving obstacle, wind, unknown clutter | AI2-THOR | ✅ 98 s, 30 evasions |
| 5 | Outdoor 8-stage with real flight dynamics | AirSim | ✅ 194 s |
| 6 | Outdoor repeat, full safety layer engaged | AirSim | ✅ 112 s, 20 interventions |
| 7 | **Two-world**: indoor → window exit → outdoor | AI2-THOR + AirSim | ✅ 12 stages |
| 8 | **Full stack flown by real ArduPilot firmware** | AI2-THOR + SITL | ✅ armed, climbed, observed, landed |

### 7.1 Autonomy demonstration (mission 3)

This mission tests the hardest capability in the system: **self-directed goal selection** — the
objective deliberately specifies *no* target and *no* landing site, so both must be chosen by the
system from what it actually perceives. The objective was phrased in cinematographic terms at the
time (that being the original application focus); the capability under test is domain-independent
and applies equally to inspection or survey targets.

Given only *"choose the most cinematic subject yourself, film it, then choose a safe landing spot and land"*:

1. Explored for 12 s, detector catalogued: television 0.97, sofa 0.82, floor lamp 0.81, armchair, desk lamp, shelf, pillow
2. The strategic model, given **only the objects actually detected**, selected the **floor lamp** — *"creates mood, contrast, and depth with light, fundamental to cinematography"* — over the obvious sofa
3. Approached and orbited it under continuous observation for 25 s
4. Scored **126 candidate floor patches** from its own sensor model and landed on the one with the greatest all-round clearance (140 cm)

### 7.2 Live-system performance (mission on real FPV video)

Every stage measured across 30 frames with hardware-realistic sensor simulation:

| Stage | Result |
|---|---|
| Perception (valid metric clearance) | 30/30 |
| Pilot intent validity | 30/30 |
| Movement safe (≤ clearance cap) | 30/30 |
| Movement sensible | 30/30 |
| Goal layer correct | 30/30 |
| Bridge indoor RC override exact | 30/30 |
| Bridge outdoor GUIDED setpoint | 30/30 |
| Errors | 0 |
| **Loop latency** | **236 ms median, 136 ms best (4.2 Hz)** |

### 7.3 Fault caught: orbit collision 🔬

An outdoor mission contacted a tree. Three contributing causes:

1. The harness supplied fabricated "clear" values for lateral sensors
2. The orbit stage commands sideways motion — exactly the unsensed axis
3. Scripted stage velocities bypassed the production clearance clamp

The transferable finding is physical: **a planar LiDAR cannot see a tree canopy, and during an orbit the camera faces the subject rather than the direction of travel.** This is a genuine blind spot on the aircraft and is why the upward-angled ToF pair matters. After wiring real 360° clearances into the pilot and applying the directional clamp to all commanded velocities, the repeat mission completed with 20 recorded proximity interventions and no contact.

---

## 8. Component benchmarks

### 8.1 Pilot model selection
Matched prompts on real FPV frames, scoring whether the selected direction matched the most open navigable sector:

| Model | Correct |
|---|---|
| Qwen2.5-VL-7B-Instruct-AWQ | **44/45 (98%)** |
| SpaceThinker-3B (spatial LoRA) | 12/45 (27%) |

In full mission simulation the 3B model produced correct written reasoning while emitting zero movement and zero gimbal commands on every step — a capability ceiling, not a prompting deficiency.

### 8.2 Hybrid intent
| Output mode | Correct decisions | Latency |
|---|---|---|
| Raw velocity vectors | 5% | 440 ms |
| **Categorical + code geometry** | **100%** | **209 ms** |

Decisions were compared before and after optimisation on identical frames to confirm speed was not gained at the cost of quality — decisions were identical.

### 8.3 TensorRT acceleration
| Model | PyTorch | TensorRT FP16 | Speedup |
|---|---|---|---|
| YOLOv8s | 17.2 ms | 7.3 ms | 2.35× |
| YOLO-World (production) | 24.0 ms | 11.5 ms | 2.10× |

Engine class names were verified identical to the production navigation vocabulary.

### 8.4 LiDAR SLAM
Virtual aircraft in a simulated 6×6 m room, receiving only noisy odometry and LiDAR returns, **no ground-truth position supplied**: **10 cm mean drift, 12 cm maximum, 9 cm final.**

---

## 9. Reproducing

```bash
# Layer 1
python -m py_compile ai/camera_brain/laptop_ai/*.py raxda_bridge/*.py

# Layers 2–4
python tests/test_local_command_path.py     # 37 checks
python tests/verify_links.py                # 44 checks
python tests/test_anchor_memory.py          # 11 checks

# Layer 5 — requires ArduCopter SITL binary
python tests/sitl_bridge_test.py            # 13 checks
python tests/sitl_mission_test.py           # 10 checks
```

⚠️ **Environment notes.** SITL tests must run in the foreground within a single shell session; detached processes are terminated when the shell exits. On Windows hosts, UTF-8 output encoding must be forced (`PYTHONUTF8=1`) or module import fails on non-ASCII log characters.

---

## 10. What verification does *not* establish

Stated explicitly, because the distinction matters:

| Verified | Not verified |
|---|---|
| Commands produce correct firmware responses | Firmware produces correct **aircraft** behaviour in air |
| Sensor fusion mathematics | Real-world sensor noise, vibration, lighting |
| LiDAR ring reaches and is ingested by the FC | BendyRuler physically deviating a route around an obstacle |
| SLAM accuracy in simulation | EKF fusion of that estimate on hardware |
| Ring direction convention in code | Ring **orientation calibration** against the physical mounting |
| Control logic under simulated wind | Behaviour in real gusts, turbulence and ground effect |

**No result in this document was obtained in powered flight.** Simulation and firmware-in-the-loop testing reduce risk substantially; they do not eliminate it. See [SAFETY.md](../SAFETY.md).
