# Build Journal

A chronological engineering record of the Autonomous AI Drone: what was built, in what order, what failed, why it failed, and how each failure was resolved.

This document deliberately records failures alongside successes. Several of the most important architectural decisions in this system exist *because* something broke first, and the root-cause analyses are more instructive than the final code.

**Conventions**
- Each phase states an **Objective**, the **Work** performed, **Problems** encountered with root cause, and the **Outcome**.
- 🔬 marks a finding that changed the architecture.
- ⚠️ marks a lesson worth carrying to any similar project.

---

## Phase 0 — Architecture definition

**Objective.** Establish a control architecture capable of executing natural-language mission objectives without scripted flight paths.

**Work.** Three candidate architectures were evaluated:

| Candidate | Assessment |
|---|---|
| Single cloud model in the control loop | Rejected — network latency inside a flight control loop is unacceptable and creates a single point of failure. |
| Single local model doing strategy and control | Rejected — models small enough to run locally at 4 Hz lack the reasoning depth for creative direction. |
| **Two-brain split** | **Selected** — cloud model reasons about intent once per objective; local model handles continuous control. |

A governing constraint was adopted early and never relaxed: **no maneuver may be hardcoded.** Orbits, follows and approaches must emerge from model intent, otherwise the system is a macro player rather than an autonomous agent.

A second constraint followed from flight-safety reasoning: **the AI must not duplicate the flight controller's responsibilities.** Stabilisation, wind rejection and position hold are solved problems in ArduPilot and are handled there. The AI selects intent and flight *mode*; the FC executes.

**Outcome.** Two-brain architecture with strict separation between semantic decision-making (model) and geometric execution (code).

---

## Phase 1 — Hardware bring-up

**Objective.** Establish a working airframe with a companion computer, flight controller, and sensor hub.

**Work.**
- ESP32 firmware developed as a sensor hub: 4× VL53L1X time-of-flight sensors behind a PCA9548A I²C multiplexer, MPU6050 IMU, addressable LED status, gimbal servo control, UDP telemetry.
- Radxa Cubie A7Z configured as companion computer: UART to the flight controller at 57600 baud (`/dev/ttyAS0`), USB LiDAR, WiFi via phone hotspot.
- ArduCopter configured for the airframe.

**Problems.**

*Motors would not spin under test.* Two causes in sequence: `FRAME_CLASS`/`FRAME_TYPE` were unset (resolved: Quad X = 1/1, requires FC reboot), then motor test was refused for a missing safety switch. The airframe has no physical safety button, so `BRD_SAFETY_DEFLT=0` was set — the correct configuration for an autonomous aircraft.

*Servo power fault.* SG90 servos drew in excess of 5 A, browned out the system in a reset loop and burned a JST harness. A 2200 µF capacitor across the servo rail resolved the brownouts; the servos were subsequently removed. 🔬 **This event is the origin of the power-integrity problem documented in Phase 16** — it was the first symptom of a fault that would eventually destroy five companion computers.

*Total I²C bus failure.* All five sensors behind the PCA9548A stopped responding. Diagnosis proceeded by elimination: a full 126-address bus scan returned zero acknowledgements, and a single MPU6050 wired directly with internal pull-ups also returned nothing. ⚠️ **A completely silent bus is not a missing device — it is a broken bus.** A missing multiplexer would still allow other devices to acknowledge. Combined with the timing (immediately after the 5 A servo fault) the evidence pointed to bus or controller damage rather than a failed component.

**Outcome.** Flight controller, companion computer and LiDAR operational. The ESP32 ToF array was set aside; LiDAR and camera depth were promoted to primary obstacle sensing, which proved architecturally beneficial.

---

## Phase 2 — Communications

**Objective.** Reliable, low-latency links between ground station, companion computer and mobile application.

**Work.** Tailscale mesh networking over a 2.4 GHz phone hotspot; WebSocket telemetry and command channel on port 8000; UDP for ESP32 telemetry.

**Problems.**

*Intermittent link loss.* The Radxa's WiFi adapter proved unreliable when switching networks, requiring a forced rescan and connect-by-UUID (connect-by-name collided with duplicate profiles). The hotspot must be 2.4 GHz — both the ESP32 and the Radxa are 2.4 GHz only, and a 5 GHz hotspot is simply invisible to them.

*Tailscale cannot carry video.* Extensive testing established that the relay path throttles UDP sufficiently to make video unusable. 🔬 **Tailscale was reclassified as control-plane only**, and video moved to a WebRTC path with direct peer connection.

⚠️ A 4G dongle was found to hijack the default route and break all TCP traffic including Tailscale; it remains unplugged.

**Outcome.** Stable control plane with a documented separation between control and media transport.

---

## Phase 3 — Video pipeline

**Objective.** Low-latency video from the GoPro to both the AI stack and the operator application.

**Work.** GoPro HERO12 connected over USB, presenting as a CDC-NCM network device.

**Problems.**

*No video despite a successful start command.* The webcam API returned success, yet UDP capture on port 8554 showed zero packets. 🔬 **Root cause: the HERO12 webcam mode is RTSP pull, not UDP push.** The correct sequence is to start webcam mode via the control API and then pull `rtsp://<gopro>:554/live`. This single misunderstanding had blocked video for an extended period.

*Browser playback stalled indefinitely.* The GoPro emits non-standard H.264 (packetization mode 0, B-frames present) that WebRTC and HLS cannot decode. Resolved by re-encoding with `libx264 -bf 0` to strip B-frames. Hardware encoding on the Cubie A7Z is non-functional (confirmed against vendor forums), so software encoding is used — the A76 cores sustain 720p at real time with roughly 50–100 ms added latency.

*15-second video latency.* WebRTC had been configured for TCP transport. Switching MediaMTX to UDP reduced latency to approximately 1 second.

**Outcome.** GoPro → ffmpeg (B-frame strip) → MediaMTX → WebRTC to the application; RTSP direct to the AI stack at lower latency.

---

## Phase 4 — Ground application

**Objective.** Operator interface for telemetry, video, commands and mission planning.

**Work.** Flutter application with WebSocket telemetry, WebView-hosted WebRTC video, 3D attitude visualisation, and a map-based mission planner.

**Problems.** Rather than continue guessing at failures, the phone was USB-debugged with `adb logcat`. This changed the debugging economics immediately and surfaced several distinct faults:

🔬 *All flight-controller telemetry was being discarded.* In the bridge's FC read loop, the entire message-parsing block sat **after a `continue` statement inside an exception handler** — unreachable code. The bridge was reading every MAVLink message from a perfectly healthy flight controller and throwing all of it away. The FC had been streaming 20+ message types at 5 Hz the whole time. ⚠️ **A silent parsing failure is indistinguishable from a hardware fault until you instrument the boundary.**

*Application crashed on reconnect.* `disconnect()` closed the WebSocket with status 1001 (`goingAway`), which Dart forbids — only 1000 and 3000–4999 are permitted. The thrown exception cascaded into `connect()` and blanked the control screen.

*Every phone request returned a server error.* An `aiohttp` import was function-local, raising `NameError` on each request; moved to module scope.

*Slow clients were being dropped.* The telemetry broadcast dropped any client exceeding a 2-second timeout, which disconnected the phone under load. Timeout raised and disconnect-on-timeout removed.

**Outcome.** Fully functional ground application. ⚠️ The general lesson — *instrument the boundary before theorising* — was applied deliberately in every subsequent debugging phase.

---

## Phase 5 — First AI integration

**Objective.** Bring the full AI stack online against live sensor and video data.

**Problems.** Three faults, all masked by a dead camera:

*Empty spatial map.* `director_core.start()` registered a packet handler but never called `ws.connect()`. The messaging client connected lazily on first send, and the first send was always a vision-driven gimbal command — so with the camera dead, the socket never opened and no LiDAR data ever arrived. Resolved by connecting explicitly at startup.

*Every command became a hover.* `process_job` grabbed a camera frame and returned a hover if none was available — before executing the movement and before sending a response. Resolved by supporting sensor-only planning on LiDAR, ToF and IMU when no camera frame exists.

*Planner crashed on first real execution.* Once the no-camera bail-out was removed, the planner ran for the first time and failed serially: an `async` function was passed to `asyncio.to_thread` (returning an un-awaited coroutine); the entire Gemini path was gated behind an optional module that was never installed, silently falling through to a legacy OpenAI path with no API key; and two conditionally assigned attributes were referenced unconditionally.

⚠️ **Four latent defects were hidden behind one early return.** Defensive bail-outs that skip the main path suppress the errors that would otherwise reveal downstream bugs.

**Outcome.** Full stack operational end to end.

---

## Phase 6 — Pilot model selection

**Objective.** Select the local pilot model on evidence rather than assumption.

**Work.** A head-to-head evaluation was constructed on real FPV frames with matched prompts, measuring whether the selected direction corresponded to the most open navigable sector.

| Model | Correct direction selection |
|---|---|
| Qwen2.5-VL-7B-Instruct-AWQ | **44/45 (98%)** |
| SpaceThinker-3B (spatial LoRA) | 12/45 (27%) |

A realistic mission simulation was also built — an L-shaped darkened room, obstacles, a rock-strewn landing target, AI-controlled gimbal, and the full angled-ToF sensor suite. The 3B model produced **correct written reasoning** ("pan left to scan the room, tilt down to look at the floor") while emitting zero movement and zero gimbal commands on every single step. It would not even turn its own camera.

🔬 **Conclusion: the 3B limitation is a capability ceiling, not a prompting problem.** Available spatial-reasoning LoRA adapters did not close the gap.

**Outcome.** Qwen2.5-VL-7B-AWQ adopted as the production pilot, served through vLLM with `awq_marlin` 4-bit quantisation to fit alongside the perception stack in 12 GB of VRAM.

---

## Phase 7 — The say–do gap and hybrid intent 🔬

**Objective.** Determine why a model with 98% directional accuracy still produced poor flight behaviour.

**Work.** Decisions were audited by separating the model's stated reasoning from its emitted numbers. The result was unambiguous: the model reasoned correctly about space and then emitted velocity vectors that contradicted its own reasoning — stating that the left was open and commanding motion into the obstacle. Measured end-to-end correctness of raw velocity output was **5%**.

This is a grounding failure, not a comprehension failure. The model's competence is categorical; continuous numeric output is not reliable at this scale.

**Resolution — hybrid intent.** The control output was restructured so that the model emits only categorical intent, and deterministic code performs all geometry:

- `head` — one of eight body-frame azimuths, or hover
- `climb` — one of five elevation slopes
- `pace` — a speed qualifier
- `yaw` — a yaw *behaviour* (`face_travel`, `face_subject`, `scan_left/right`, `hold`)

Code converts these into a spherical velocity vector, clamps it to the aircraft's live envelope read from flight-controller parameters, applies a stopping-distance speed cap from measured clearance, and — critically — **re-steers to the most forward open sector if the model's preferred direction is blocked.** The model's choice is treated as a preference, not a command.

**Results.**

| Metric | Before | After |
|---|---|---|
| Correct navigation decisions | 5% | **100%** |
| Decision latency | 440 ms | **209 ms** |
| Output tokens per decision | ~40 | ~6 |

🔬 **Reliability and latency improved simultaneously** — an unusual outcome that follows directly from the fact that short categorical outputs are both easier for the model to get right and faster to generate.

Complex behaviour was verified to *emerge*: sustained lateral intent combined with `face_subject` yaw produces a geometrically correct orbit with no orbit routine in the codebase.

---

## Phase 8 — Latency engineering

**Objective.** Bring the complete loop — including vision-language inference — within 0.2–0.3 s without degrading decision quality.

**Work.**
1. **Persistent inference server.** vLLM loads the model once and captures CUDA graphs once, rather than paying initialisation on every decision.
2. **Compact output.** The hybrid-intent change reduced output tokens by roughly 85%; at short generations, latency is dominated by token count.
3. **Threaded perception.** Detection and depth run on a background thread that continuously refreshes shared state. Loop time becomes *max(perception, inference)* rather than their sum — perception refreshed 95 times across 29 decision ticks during measurement.
4. **Prefix caching** for the large static pilot prompt.

**Verification.** Decisions were compared before and after optimisation on identical frames to confirm that speed was not obtained at the cost of quality — decisions were identical.

**Outcome.** **236 ms median** end-to-end (4.2 Hz), 136 ms best case. ⚠️ First-run measurements after server start are 2–3× slower due to compilation warm-up and must be discarded.

---

## Phase 9 — Sensor fusion and spatial awareness

**Objective.** Build a unified obstacle representation from heterogeneous sensors.

**Work.** A 2.5D occupancy grid (20 cm cells, 10 m extent) fusing 360° LiDAR, time-of-flight, and monocular metric depth. Relative-depth heuristics were replaced with **Depth-Anything-V2-Metric**, giving true metric distances. Object detection moved to **YOLO-World** open-vocabulary detection, which — unlike a fixed COCO vocabulary — can detect navigation-relevant classes such as *doorway*, *wall* and *pillar*.

**Problems.**

🔬 *A critical audit finding: the resolver's safety logic was inert on hardware.* The pilot's clearance inputs were sourced from the ESP32 time-of-flight sensors — which had been dead since Phase 1 and defaulted to a "clear" sentinel value. Every clearance-dependent safeguard (speed capping, direction correction, goal steering) was reading phantom open space and doing nothing. The system had been tested with synthetic clearance values in harnesses, which masked the fault completely.

Resolved by sourcing pilot clearances from the fused spatial grid (real LiDAR plus metric depth). Before the fix, the resolver commanded 0.6 m/s into a wall 45 cm away; after, it re-steered.

⚠️ **Test harnesses that synthesise inputs can validate logic while hiding the fact that the logic receives no real data.** Verification must trace the data path, not merely exercise the function.

*Direction-correction bias.* The re-steering logic originally chose the maximum-clearance sector, causing the aircraft to reflexively reverse. Changed to prefer the most *forward* open sector, reversing only when genuinely boxed in.

*Detector label mismatch.* Object labels were being read from an unrelated classifier rather than the active detector, corrupting mission target matching. A synonym map was also added (sofa/couch, tv/television) to reconcile natural-language mission text with detector vocabulary.

**Outcome.** Verified fusion feeding genuinely live obstacle data to the pilot.

---

## Phase 10 — Flight modes, return policy and wind

**Objective.** Let the AI use the flight controller's capabilities rather than reimplement them.

**Work.** An initial implementation had the AI estimate wind and derate its own speed. This was **reverted by design decision**: with a GPS fix, ArduPilot's position controller already rejects wind while holding commanded velocity. Duplicating that in the AI layer produces two controllers fighting the same disturbance. Wind is now presented to the model as context only.

The AI instead selects *flight modes*: GUIDED while actively flying, LOITER for sustained holds (the FC holds position precisely and the AI stops streaming zero velocities), and return modes on failsafe. Mode changes use hysteresis to prevent oscillation and are gated on a valid position fix.

Return behaviour was made fully operator-defined: destination (home / operator position / land in place) and battery threshold both come from application settings, with automatic fallback to land-in-place when no position fix is available.

**Outcome.** Clean division of labour: AI chooses intent and mode; ArduPilot executes stabilisation, wind rejection and return.

---

## Phase 11 — GPS-denied indoor positioning

**Objective.** Enable indoor flight, where `PreArm: Need Position Estimate` blocks all position-dependent modes.

**Work.** A 2D LiDAR SLAM module was implemented — scan-to-map correlative hill-climbing correcting a dead-reckoned prior, with heading from the flight-controller compass. Output is published to the EKF as `VISION_POSITION_ESTIMATE`.

**Validation.** A virtual drone was flown in a simulated 6×6 m room receiving only noisy odometry and LiDAR returns, with no ground-truth position: **10 cm mean drift, 12 cm maximum**.

⚠️ The corresponding EKF source parameters (`EK3_SRC1_POSXY=6` ExternalNav) are **deliberately not auto-configured** by the bridge. Automatically switching EKF sources on connect would break outdoor GPS flight. They are documented as a deliberate operator step, ideally on a switchable source set.

**Outcome.** Indoor positioning implemented and validated in simulation; hardware EKF fusion remains to be tested.

---

## Phase 12 — The regression event and forensic recovery 🔬

**Objective.** Investigate a user report that application controls which had previously worked no longer functioned.

This phase is retained in full because the failure mode is common in embedded projects and the recovery method is reusable.

**The report.** The operator stated that the application's ARM button had worked previously over the local link. Initial code analysis suggested it could not have — the local command path routed named commands into a numeric-only executor that would ignore them.

**The investigation.** Rather than defend the analysis, the claim was tested against primary evidence: session transcripts containing verbatim runtime logs. The operator was correct. A log from several weeks earlier showed the complete working sequence:

```
INBOUND type=command action=ARM payload=ARM
LOCAL ARM: STABILIZE + arm (no GPS)
... Armed=128
```

The handler that produced those lines was **absent from the currently deployed bridge.**

**Root cause.** Approximately two weeks of fixes had been applied directly to the companion computer over SSH and never committed to version control. A subsequent full-file refactor replaced the deployed file wholesale, silently destroying every one of them.

**Recovered and restored:**

| Lost capability | Consequence while missing |
|---|---|
| `cmd_vel` packet acceptance | **The entire autonomous velocity stream was ignored** — the AI could not move the aircraft at all over the local link |
| No-GPS ALT_HOLD attitude-override conversion | Indoor autonomous flight impossible |
| AI job relay between application and ground station | Natural-language commands never reached the AI; responses never returned |
| Named local commands (ARM, LAND, RTL, modes) | Application buttons inert — and worse, a side effect force-switched the aircraft to GUIDED even for ignored commands |
| Landing disarm backstop and command lockout | Landing could be overridden by a stale velocity stream |
| GPS-gated returns | A return without a position fix attempted a blind RTL instead of landing in place |
| True armed-state reporting | `armed` was derived from a gyro-health bit that is always 1 — the ground station believed it had taken off when the motors had never moved |
| Throttle-deadzone and disarm-delay parameters | Small climb commands were swallowed; auto-disarm interrupted sequences |

**Recovery method.** Session transcripts contain verbatim tool invocations, including the full text of every patch script. Each lost fix was recovered from those records, re-applied to the worktree copy, and verified — 35 assertions covering exact stick values, lockout behaviour, GPS gating and relay routing.

⚠️ **Lessons.**
1. **On-device edits that never reach version control will eventually be destroyed.** Port every hotfix to the repository immediately.
2. **When a user contradicts your analysis, test the claim before defending the analysis.** The operator's recollection was accurate; the code reading was accurate *for the current file*. Both were true, and the gap between them was the bug.
3. Primary-evidence logs are worth more than reasoning about source code.

---

## Phase 13 — Firmware-in-the-loop validation 🔬

**Objective.** Validate the bridge against real flight-control firmware before committing to hardware.

**Work.** ArduCopter SITL — the same firmware family that runs on the aircraft — was executed locally, and the **actual production bridge methods** were bound to that MAVLink connection. Commands are issued exactly as the application issues them, and the firmware's real response is asserted.

**Faults found before they reached hardware:**

🔬 *Takeoff would have been silently refused.* `NAV_TAKEOFF` is only valid in GUIDED, but the application's split-arm sequence arms in STABILIZE. The firmware rejected the command with `result 4`. On the aircraft this would have presented as "the drone arms but will not take off" — a difficult field diagnosis. Fixed by ensuring GUIDED before commanding takeoff.

🔬 *The EKF readiness signal was mis-identified.* GUIDED is refused with `requires position` for several seconds *after* a GPS 3D fix is reported, until the EKF actually adopts GPS for lateral position. The operator's existing field heuristic ("wait for the green fix") was correct but imprecise; the true readiness event is `EKF3 using GPS`, which arrives after `origin set`. Sequencing now gates on that event.

**Final result: 13/13**, including arm, all mode transitions, a real parameter write, takeoff, **commanded velocity producing measured ground speed of 2.98 m/s against a 3.0 m/s command**, RTL, landing with auto-disarm, and the GPS-denied attitude-override path.

---

## Phase 14 — Simulation campaign

**Objective.** Validate complete missions with the full AI stack before flight.

**Work.** Seven missions were flown across three simulation environments:

| Mission | Environment | Result |
|---|---|---|
| Dark-room search, observation and hazard-avoiding landing | Custom raycast sim | ✅ 56 s |
| Photorealistic target search and observation | AI2-THOR | ✅ 38 s |
| **Full autonomy** — AI selects its own subject *and* landing site | AI2-THOR | ✅ 46 s |
| 8-stage indoor stress test with moving obstacle and wind | AI2-THOR | ✅ 98 s, 30 evasions |
| Outdoor 8-stage mission with real flight dynamics | AirSim | ✅ 194 s |
| Outdoor repeat with full safety layer engaged | AirSim | ✅ 112 s, 20 proximity interventions |
| **Two-world mission** — indoor, window exit, outdoor continuation | AI2-THOR → AirSim | ✅ 12 stages |
| **Full stack flown by real ArduPilot firmware** | AI2-THOR + SITL | ✅ armed, climbed, observed, landed |

In the full-autonomy mission the strategic model was given only the object catalogue the drone had actually detected, and selected the floor lamp over the obvious sofa, reasoning that it *"creates mood, contrast, and depth with light."* The aircraft then scored 126 candidate floor patches from its own sensor model and landed on the one with the greatest all-round clearance.

**Problems.**

*An outdoor mission contacted a tree.* Root cause was threefold and instructive: the simulation harness supplied fabricated "clear" values for the lateral sensors; the orbit stage commanded sideways motion — precisely the unsensed axis; and scripted stage velocities bypassed the production clearance clamp. 🔬 **The genuine transferable finding is that a planar LiDAR cannot see a tree canopy, and during an orbit the camera faces the subject rather than the direction of travel.** This is a real blind spot on the aircraft and is the reason the upward-angled ToF pair matters. After wiring real 360° clearances into the pilot and applying the directional clamp to every commanded velocity, the repeat mission completed with 20 recorded proximity interventions and no contact.

*RC overrides expired mid-flight.* ArduPilot invalidates RC overrides after 3 seconds. The simulation tick was slower than that, so climb commands lapsed between ticks. ⚠️ **This was corrected in simulation only** — on the real aircraft the ground station streams at roughly 15 Hz, and the 3-second expiry is a valuable failsafe that neutralises the sticks if the AI stalls. It must not be raised in production.

---

## Phase 15 — Perception acceleration

**Objective.** Reduce perception cost to leave GPU headroom for the pilot model.

**Work.** Detection models were exported to TensorRT FP16 engines built for the target GPU.

| Model | PyTorch | TensorRT | Speedup |
|---|---|---|---|
| YOLOv8s | 17.2 ms | 7.3 ms | 2.35× |
| YOLO-World (production detector) | 24.0 ms | 11.5 ms | 2.10× |

⚠️ An initial export optimised the wrong model — the fallback detector rather than the open-vocabulary detector actually used in production. Open-vocabulary models require their class vocabulary to be **baked in at export time**; the exported engine's class names were verified to match the production navigation vocabulary exactly. Changing the class list requires re-export.

---

## Phase 16 — Hardware deployment and the power-integrity investigation 🔬

**Objective.** Deploy the validated stack to the aircraft.

**Work and results.** The restored bridge was deployed and verified live on the aircraft:
- 360° LiDAR ring streaming to the flight controller (37–41 of 72 sectors populated, forward wall measured at 60 cm)
- `PRX1_TYPE=2` set and persisted across a firmware reboot
- Live aircraft envelope parameters flowing to the AI
- Production TensorRT detector loaded, with the safety layer producing correct braking decisions from real LiDAR returns

🔬 **A long-standing mystery was resolved.** For weeks, `rangefinder1` had read zero in the ground control station despite the companion computer provably transmitting valid distance data. Isolation testing showed the reading returned when proximity was disabled and vanished when it was re-enabled. The behaviour is **correct and by design**: when the proximity library is configured to source from the rangefinder, it *claims* that sensor, and the value moves from the rangefinder field into the proximity subsystem. Nothing was ever broken. Reconfiguring proximity to consume the MAVLink `OBSTACLE_DISTANCE` ring frees the rangefinder and provides direction-aware avoidance. Ingestion was confirmed by injecting a known 87 cm distance and observing the firmware re-emit it.

**The companion-computer failures.** Over the project, **five single-board computers failed** — three Radxa Zero 3W and two Cubie A7Z. Four stopped responding with no visible damage; one showed a clear short. The network registry preserved an unambiguous timeline of roughly one failure per week of active use.

**Root cause.** The operator reported that two failures coincided exactly with **JST wiring from the 5 V power module burning out**. The mechanism follows directly:

1. JST connectors are rated for approximately 2–3 A. The rail carried the companion computer (2–3 A with LiDAR and camera on its USB) plus additional electronics — at or beyond the connector's rating.
2. Contact resistance rose with heat, which produced more heat — classic thermal runaway in a connector.
3. **As the connector failed, the ground pin released before the 5 V pin.** Return current then had only the data lines — UART to the flight controller, USB to the LiDAR — as a path, destroying the I/O domain and power-management circuitry.

This explains the silent, damage-free failures precisely, and very likely explains the Phase 1 I²C multiplexer death as well: a related over-current event on the same rail.

**Required mitigation** (documented in `docs/HARDWARE.md`, cost under USD 10):
- Dedicated 5 V buck converter for the companion computer, independent of the flight-controller rail
- **XT30 or directly soldered 20 AWG for all power — JST reserved for signals only**
- Inline fuse and a TVS diode at the board input
- Camera powered independently, never from the companion computer's USB
- Nylon standoffs — carbon-fibre frames are electrically conductive

⚠️ **The broader lesson: repeated "random" failures of identical components are almost never random.** The pattern was visible in the timeline for weeks before the root cause was pursued.

A sixth board was commissioned by transferring the SD card, which restored the complete software stack and network identity unchanged.

---

## Current status

| Area | Status |
|---|---|
| AI stack, resolver, safety logic | ✅ Complete, verified |
| Bridge ↔ firmware command paths | ✅ Verified against real ArduPilot firmware |
| Sensor fusion, LiDAR → FC ingestion | ✅ Confirmed on hardware |
| Ground application | ✅ Functional |
| Hardware integration | 🟡 Partial — awaiting power-system rebuild |
| Powered flight | 🔴 Not yet performed |

---

## Scope note: cinematography deferred

The system was originally framed around autonomous aerial cinematography, and several capabilities
retain that heritage — a gimbal the AI aims itself, `face_subject` yaw tracking, and emergent orbits
at a maintained radius. Mission 3 above demonstrates the system selecting a subject on aesthetic
reasoning and filming it unaided.

The project focus has since been generalised to **autonomous completion of complex multi-stage
missions**, of which cinematography is one application. The reasoning is practical: subject
tracking, obstacle-aware approach and precision landing are prerequisites for *any* task-completing
drone, whereas shot grammar, jerk-limited camera trajectories and multi-shot continuity are a
specialised layer that only becomes worth building on top of proven autonomy. A drone that cannot
reliably complete an inspection cannot reliably film one either.

Cinematography-specific work is tracked in the project roadmap as a planned upgrade.

---

## Engineering principles derived from this project

1. **Split responsibilities along competence boundaries.** Language models are reliable at semantics and unreliable at continuous geometry. Architect to that boundary rather than fighting it.
2. **Verify the data path, not only the function.** A function can pass every test while receiving no real input.
3. **Instrument the boundary before theorising.** Attaching a debugger to the phone, and reading the firmware's own status messages, each collapsed weeks of speculation into minutes.
4. **Test user claims before defending your analysis.** The most significant regression in this project was found because a contradicting recollection was checked against primary evidence.
5. **Every uncommitted hotfix is already lost.** Version control is not bureaucracy; it is the only durable record.
6. **Simulate against the real artefact.** Running the actual flight firmware caught two failures that would have been expensive and confusing to diagnose in the field.
7. **Repeated identical failures indicate a systemic cause.** Investigate the pattern, not the instance.
8. **Do not reimplement what the platform already does well.** Wind rejection and stabilisation belong to the flight controller.
