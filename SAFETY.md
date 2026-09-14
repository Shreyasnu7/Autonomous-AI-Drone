# Safety

**Read this document in full before operating this system.**

This software commands a multirotor aircraft with exposed rotating propellers, capable of causing serious injury, death and property damage. It is provided without warranty of any kind (Apache License §§7–8). The operator bears sole responsibility for safe operation and for compliance with all applicable aviation regulations.

---

## 1. Maturity statement

**Flight testing is underway. The autonomous stack is not yet cleared for unsupervised flight.**

The aircraft is flying under pilot control, and autonomous behaviours are being validated against it one stage at a time (§6). Verification prior to flight consisted of simulation and software-in-the-loop testing against real flight-control firmware. That reduced risk substantially — several failures that would have occurred in the field were caught and corrected — but it does not establish airworthiness.

**Treat any behaviour whose flight-test stage has not been recorded as cleared as unproven, and fly it as a first-time behaviour.** The table below reflects what bench and simulation work establishes; flight data supersedes it only for stages actually flown.

| Verified | Not verified |
|---|---|
| Command paths against real firmware | Aircraft behaviour in air |
| Sensor fusion mathematics | Real sensor noise, vibration, lighting |
| LiDAR ingestion by the flight controller | Avoidance physically deviating a flight path |
| Control logic under simulated wind | Real gusts, turbulence, ground effect |
| Ring direction convention in code | Ring orientation against the physical mounting |

Treat every first-time behaviour as unproven.

---

## 2. Non-negotiable rules

1. **Propellers off** for every bench test, every first run of new software, and every configuration change.
2. **Maintain a manual override.** The transmitter must be powered, bound, and within reach at all times. The operator must be able to take control or disarm instantly.
3. **Never fly over people, animals, vehicles, or property you are unwilling to lose.**
4. **Maintain visual line of sight.**
5. **Comply with local aviation law.** Registration, altitude limits, airspace restrictions and permissions are the operator's responsibility.
6. **Do not fly an unproven configuration in a confined space.** Indoor autonomous flight is the highest-risk mode — no GPS, short distances, hard surfaces.

---

## 3. Pre-flight procedure

### 3.1 Bench (propellers removed)

| # | Check | Pass condition |
|---|---|---|
| 1 | Companion computer online | Reachable; bridge service active |
| 2 | Flight-controller link | Heartbeat present; telemetry populated |
| 3 | LiDAR ring | `LIDAR RING n/72 sectors` with plausible distances |
| 4 | **Ring orientation** | Obstacle placed ahead appears **at the top** of the proximity display |
| 5 | Battery voltage | Reported voltage matches a multimeter reading |
| 6 | Arming | Motors arm and disarm on command |
| 7 | Motor test | All four spin, correct direction and order |
| 8 | Return policy | Application settings reflected in bridge configuration |
| 9 | Failsafe | Disconnect the link — aircraft responds as configured |

⚠️ **Check 4 is mandatory before any avoidance-dependent flight.** The ring's direction convention is configurable and has **not** been calibrated against the physical LiDAR mounting. If obstacles appear mirrored or rotated, avoidance will steer the wrong way. Adjust `LIDAR_DIR` / `LIDAR_YAW_OFFSET_DEG` until the display matches reality.

### 3.2 Field

| # | Check | Pass condition |
|---|---|---|
| 1 | Area | Clear of people, obstacles, airspace restrictions |
| 2 | Weather | Wind within aircraft capability; no precipitation |
| 3 | GPS (outdoor) | 3D fix, 8+ satellites, **stable before arming** |
| 4 | EKF readiness | Wait for the EKF to adopt GPS for position — this occurs *seconds after* the fix is first reported |
| 5 | Battery | Fully charged; return threshold configured |
| 6 | Transmitter | Powered, bound, mode switches verified |
| 7 | Abort plan | Operator knows the disarm action before arming |

---

## 4. Staged flight-test programme

Do not skip stages. Each validates something the next depends on.

| Stage | Test | Validates |
|---|---|---|
| 0 | Bench, props off | Links, sensors, arming, ring orientation |
| 1 | Manual hover, props on, transmitter only | Airframe, tune, no AI involved |
| 2 | Manual flight with the bridge running | Telemetry and sensor streams under vibration |
| 3 | **Assisted hover** — AI commanding, operator on sticks | AI velocity path, at low altitude, ready to override |
| 4 | Assisted low-speed translation, open area | Clearance caps, direction correction |
| 5 | Avoidance approach toward a **soft** obstacle | Proximity braking and path deviation |
| 6 | Short outdoor autonomous mission, GPS | Mode selection, mission execution |
| 7 | Return policy under a deliberate low battery | Failsafe behaviour |
| 8 | Indoor GPS-denied | Highest risk — only after all above pass |

---

## 5. Failure modes and responses

| Failure | System behaviour | Operator action |
|---|---|---|
| Ground-station link lost | RC overrides expire in 3 s; sticks neutralise | Take manual control |
| AI process stops | Same as above; FC remains in its current mode | Take manual control |
| Cloud model unavailable | Mission continues on last known intent | None required |
| Pilot model timeout | Safe default for that tick; safety layers remain active | Monitor |
| GPS lost outdoors | Position modes degrade; returns land in place rather than blind RTL | Take manual control |
| Obstacle inside stopping distance | Speed capped to zero; re-steer or hover | Monitor |
| Low battery at threshold | Configured return executes once (latched) | Monitor; be ready to override |
| Landing detector fails | Disarm backstop fires after the descent window | Verify motors stopped |

**RC override expiry is a safety feature.** ArduPilot invalidates overrides after 3 seconds; the ground station streams at ~15 Hz, well inside that window. If the AI stalls, the sticks neutralise rather than holding a stale command. **Do not increase `RC_OVERRIDE_TIME` on the aircraft.**

---

## 6. Known hazards specific to this system

**Planar LiDAR blind spots.** The LiDAR scans a single horizontal plane. Tree canopies, overhanging branches, tables and low obstacles are invisible to it. During an orbit the camera faces the *subject*, not the direction of travel — so the depth cone does not cover the travel axis either. **This combination caused a simulated collision with a tree.** Maintain generous clearance during orbits near vegetation or overhangs.

**Indoor flight without position aiding.** Without GPS or optical flow, the aircraft holds altitude but drifts laterally. Indoor autonomous flight is experimental.

**Power system.** Five companion computers were destroyed by a power-integrity fault. If the mitigation in [docs/HARDWARE.md §4](docs/HARDWARE.md) has not been implemented, the companion computer may fail mid-flight — taking the AI, LiDAR feed and telemetry with it. The flight controller continues to fly, but autonomy is lost instantly.

**Untested configuration changes.** Changing the detector class list requires re-exporting the TensorRT engine. Changing EKF source parameters affects arming and mode availability. Verify on the bench after any such change.

---

## 7. Emergency procedures

| Situation | Action |
|---|---|
| Uncommanded movement | Switch to a manual mode on the transmitter immediately |
| Aircraft unresponsive | Switch to LAND or RTL; if unresponsive, disarm (accepting a fall) |
| Imminent collision with a person | **Disarm immediately.** A crashed aircraft is preferable to an injury |
| Fire (battery) | Do not use water. Class D extinguisher or sand; isolate the pack |
| Flyaway | Attempt RTL; if unresponsive, disarm. Record last known position and heading |

---

## 8. Regulatory

The operator is responsible for determining and complying with applicable law, which varies by jurisdiction and changes over time. This typically includes aircraft registration, remote-pilot certification, altitude and distance limits, airspace authorisation, restrictions over people and infrastructure, and insurance.

Autonomous operation may be subject to additional restrictions beyond those applying to manually flown aircraft. Nothing in this repository constitutes authorisation to operate.

---

## 9. Reporting

Safety-relevant defects should be reported through the repository issue tracker with the tag `safety`, including firmware version, configuration, logs and reproduction steps. Please report unsafe behaviour observed in simulation as well as in flight — several defects in this system were caught in simulation precisely because they were reported rather than dismissed.
