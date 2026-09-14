# Hardware

Bill of materials, integration guidance, and power-integrity requirements.

> ⚠️ **Read §4 (Power Integrity) before wiring anything.** Five companion computers were destroyed during development by a single root cause. The mitigation costs under USD 10 and is not optional.

---

## 1. Bill of materials

### Airframe and propulsion
| Item | Specification | Notes |
|---|---|---|
| Frame | Quad X, carbon fibre | Electrically conductive — see §4.5 |
| All-up weight | 1.6–1.7 kg | Used by the AI for thrust-headroom reasoning |
| Battery | 3S LiPo, 5400 mAh | |
| Power module | Matek FCHUB-12S V2 (5 V / 5 A BEC) | **Flight controller only** — see §4 |
| Connector standard | XT60 (main), XT30 (subsystem) | Anti-spark recommended |

### Avionics
| Item | Specification | Interface |
|---|---|---|
| Flight controller | MiniPix, ArduCopter 4.6.x custom build | UART @ 57600 |
| Companion computer | Radxa Cubie A7Z | UART pins 8/10 → `/dev/ttyAS0` |
| Sensor hub | ESP32 | I²C `SDA=21`, `SCL=22`; WiFi/UDP telemetry |
| GNSS | u-blox module | UART to FC; mast standoff recommended |

### Sensors
| Item | Specification | Interface |
|---|---|---|
| LiDAR | YDLIDAR X2, 360° planar, 8 m | USB (CP210x) → `/dev/ttyUSB0` |
| Time-of-flight | 4× VL53L1X, 4 m | I²C behind PCA9548A mux (addr `0x70`) |
| IMU (auxiliary) | MPU6050 | I²C behind mux |
| Camera | GoPro HERO12 | USB CDC-NCM, RTSP pull |
| Gimbal | 2-axis, pan/tilt | Servo, ESP32-driven |

### Ground segment
| Item | Specification |
|---|---|
| Ground station | Laptop, NVIDIA RTX 5070 Ti 12 GB, CUDA 12.x, WSL2 |
| Link | Tailscale mesh over 2.4 GHz hotspot |
| Operator device | Android phone, Flutter application |

---

## 2. Companion computer integration

### 2.1 Flight controller UART
Companion pins **8 and 10** map to `/dev/ttyAS0`. Enable the corresponding device-tree overlay and reboot.

⚠️ Pin 12 is **not** a UART on the Cubie A7Z. An early attempt used a Radxa Zero-3W pinout — same physical connector, different SoC, different mapping. Always verify against the pinout for the exact board revision.

### 2.2 LiDAR
The official YDLIDAR SDK is not required; the serial protocol parser in `lidar_driver.py` is used and has proven reliable (15 consecutive scans, 100+ points each, 0.69–5.1 m range, during validation).

### 2.3 Service configuration
The bridge runs as a systemd service. **`Environment=PYTHONUNBUFFERED=1` is mandatory.**

Without it, Python block-buffers stdout under systemd and the LiDAR loop produced no journal output whatsoever — creating the false impression of a hardware failure for an extended period. The hardware and driver were healthy throughout; only the logging was invisible.

---

## 3. Flight controller configuration

### 3.1 Custom firmware
The stock 4.6.x build for 1 MB flash targets **omits the proximity and avoidance libraries entirely**. A custom build via `custom.ardupilot.org` is required, force-including:

`PRX` · `AVOID` · `AP_OAPathPlanner` (BendyRuler + Dijkstra) · `SmartRTL` · `VISO` · `Terrain` · `OpticalFlow` · `Beacon` · `Gyro-FFT` · `Precision Landing`

Excluded to fit within 1 MB: Helicopter, Plane, Parachute, Airspeed, Winch, Sprayer. Resulting image: **935 KB / 1 MB (91.3%)**.

⚠️ Reflashing custom firmware does **not** erase tuning or AutoTune results — parameters are stored separately. Back them up regardless.

### 3.2 Required parameters

| Parameter | Value | Purpose |
|---|---|---|
| `FRAME_CLASS` / `FRAME_TYPE` | 1 / 1 | Quad X (FC reboot required) |
| `BRD_SAFETY_DEFLT` | 0 | No physical safety switch |
| `RNGFND1_TYPE` | 10 | MAVLink rangefinder |
| `RNGFND1_ORIENT` | 0 | Forward |
| **`PRX1_TYPE`** | **2** | **MAVLink proximity — consumes the 72-sector ring** |
| `AVOID_ENABLE` | 2 | Proximity-based avoidance |
| `AVOID_BEHAVE` | 1 | Stop (not slide) |
| `AVOID_MARGIN` | 1.0 m | Standoff distance |
| `OA_TYPE` | 3 | Dijkstra + BendyRuler path planning |
| **`AUTO_OPTIONS`** | **3** | **Arm in AUTO + take off without a throttle raise** |
| `THR_DZ` | 0 | No ALT_HOLD throttle deadzone |
| `DISARM_DELAY` | 0 | No auto-disarm mid-sequence |

**On `PRX1_TYPE`.** Setting this to `4` (rangefinder-sourced proximity) causes the proximity library to *claim* the forward rangefinder — the value disappears from the `rangefinder1` field and moves into the proximity subsystem. This is correct behaviour, not a fault, and it consumed considerable diagnostic effort before being identified. Use `2` so proximity consumes the MAVLink ring and the rangefinder remains independently readable.

**On `AUTO_OPTIONS`.** With defaults, a mission in AUTO waits for a **pilot throttle raise** before executing the takeoff item. An application-flown aircraft has no throttle stick, so missions would arm and then sit idle indefinitely. Bit 1 permits arming in AUTO; bit 2 permits takeoff without a throttle raise.

### 3.3 Indoor EKF (optional, deliberate)
For GPS-denied flight using LiDAR SLAM:

```
EK3_SRC1_POSXY = 6      (ExternalNav)
EK3_SRC1_VELXY = 0
EK3_SRC1_POSZ  = 1      (Baro)
EK3_SRC1_YAW   = 1      (Compass)
```

⚠️ These are **not** set automatically by the bridge — doing so on connect would break outdoor GPS flight. Configure them deliberately, ideally on a switchable source set (SRC1 indoor, SRC2 outdoor) bound to a transmitter switch.

---

## 4. Power integrity 🔴

### 4.1 Failure history

Five single-board computers were destroyed during development:

| Board | Failure signature |
|---|---|
| 3× Radxa Zero 3W | No response, no visible damage |
| 1× Cubie A7Z | No response, no visible damage |
| 1× Cubie A7Z | Visible short |

Failures occurred at roughly one per week of active use. Two coincided **exactly** with JST wiring from the 5 V power module burning out.

### 4.2 Root cause

1. **Connector over-current.** JST connectors are rated for approximately 2–3 A. The rail carried the companion computer — 2–3 A with LiDAR and camera drawing from its USB — plus additional electronics. At or beyond rating.
2. **Thermal runaway.** Rising contact resistance produced heat, which raised resistance further.
3. **Ground-first disconnection.** As the connector failed, the ground pin released while 5 V remained connected. Return current then had only the data lines — UART to the flight controller, USB to the LiDAR — as a return path.
4. **Silent I/O destruction.** Current through data pins destroyed the I/O domain and power-management circuitry, producing an unresponsive board with no external damage.

The same rail had previously carried a 5 A+ servo fault, which very likely also destroyed the I²C multiplexer described in the build journal.

### 4.3 Required mitigation

| # | Requirement | Approx. cost |
|---|---|---|
| 1 | **Dedicated 5 V / 4–5 A buck converter** for the companion computer, direct from the battery. The flight-controller rail powers the FC only. | $4 |
| 2 | **XT30 or directly soldered 20 AWG silicone wire** for all power. **JST is reserved for signals permanently.** | $2 |
| 3 | **Inline 3 A fuse** on the companion feed | $1 |
| 4 | **TVS diode (SMBJ5.0A)** across 5 V/GND at the board input | $1 |
| 5 | **2200 µF electrolytic + 100 nF ceramic** at the board input | $1 |
| 6 | **Star grounding** — companion ground direct to the source lug, never daisy-chained | — |
| 7 | **Camera powered independently** (own battery or dedicated BEC) — never from companion USB | — |
| 8 | **Nylon standoffs / non-conductive tray** — carbon fibre conducts | $1 |

Total: **under USD 10**, against five destroyed boards.

### 4.4 Recommended topology

```
Battery ──┬── Matek FCHUB ──── Flight controller, ESCs
          │
          └── XT30 ── 5V/5A buck ── 3A fuse ── TVS ── caps ── Companion computer
                                                                    │
                                                              (USB) └── LiDAR only

Camera ── independent battery or dedicated BEC     [never companion USB]
```

### 4.5 Conductive frame
Carbon fibre is electrically conductive. Boards must be mounted on nylon standoffs or a non-conductive tray. A vibrating board in contact with the frame produces intermittent shorts that are extremely difficult to diagnose in flight.

### 4.6 Diagnosing an unresponsive board

"No response" is not proof of failure. Two recoverable states present identically to a dead board:

1. **SD corruption** — brownouts corrupt cards far more often than they destroy silicon.
2. **PMIC latch-up** — survives normal power cycling; clears after 10+ minutes fully unpowered.

**Procedure:** bare board, nothing attached → freshly flashed SD card → wall USB supply → observe LEDs. A USB power meter is decisive: a genuinely dead board draws ≈ 0 mA; anything drawing 100 mA+ is alive and failing to boot.

Board replacement retains the full software stack and network identity by transferring the SD card, provided the replacement is the **same model** (different SoC ⇒ incompatible kernel and device tree).

---

## 5. Sensor hub (ESP32)

### 5.1 Pin assignment
| Function | Pin |
|---|---|
| I²C SDA / SCL | 21 / 22 |
| ToF XSHUT | 23, 26, 15, 5 |
| PCA9548A address | `0x70` |

### 5.2 Firmware behaviour
The firmware self-heals: it re-checks the multiplexer every 2 seconds and re-initialises sensors live, so restoring wiring clears the error state without a reboot. Telemetry includes a live bus scan (`i2c` field) reporting all responding addresses — the primary diagnostic for I²C faults.

### 5.3 I²C fault diagnosis

A scan returning `NONE` means **the bus is broken, not that a device is missing** — a missing multiplexer would still leave other devices responding.

| Check | Method | Interpretation |
|---|---|---|
| Power | Multimeter at PCA VCC | Must read 3.3 V |
| Ground | Continuity, ESP GND ↔ sensor board GND | Must be continuous |
| Bus idle | SDA and SCL to ground | ≈ 3.3 V normal; ≈ 0 V held low; drifting = no pull-up |
| Pull-ups | External 4.7 kΩ to 3.3 V on both lines | Internal pull-ups are weak |
| Multiplexer reset | `/RESET` pin | Must be tied high or the device never acknowledges |
| **Controller pins** | Remap `Wire.begin(25, 33)`, retest with one device | Device appears ⇒ original GPIO damaged; remap permanently |

⚠️ Numerous copies of the firmware source exist across working directories. The Arduino IDE silently builds whichever file it has open, which can make edits appear to revert. Verify a unique token in the open file before uploading.

---

## 6. Communications

| Link | Transport | Notes |
|---|---|---|
| Companion ↔ FC | UART 57600 | Bridge owns the port exclusively — stop the service before any direct MAVLink tooling |
| Companion ↔ ground | WebSocket :8000 over Tailscale | Control plane only |
| Video → operator | WebRTC via MediaMTX :8889 | UDP transport (TCP adds ~15 s latency) |
| Video → AI | RTSP :8554 | Lower latency than WebRTC |
| ESP32 → companion | UDP :8888 | Broadcast + unicast |

**Constraints.** The hotspot must be **2.4 GHz** — both the ESP32 and companion are 2.4 GHz only. Tailscale is control-plane only; its relay path throttles UDP and cannot carry video. A 4G dongle hijacks the default route and breaks all TCP traffic including Tailscale.

---

## 7. Recommended upgrades

| Upgrade | Cost | Benefit |
|---|---|---|
| **Power-integrity kit (§4.3)** | **< $10** | **Prevents further board losses — highest priority** |
| Optical-flow + ToF module (e.g. MTF-01) | ~$25 | True GPS-denied position hold; removes the dominant indoor blocker |
| PCA9548A replacement + pull-ups | ~$5 | Restores four already-owned ToF sensors |
| Propeller guards | ~$12 | Indoor testing without airframe risk |
| Dedicated companion WiFi adapter | ~$12 | Removes the unreliable onboard adapter from the link path |
| GPS mast standoff | ~$5 | Reduces ESC noise; faster fix, cleaner heading |
| Panoramic camera | ~$150+ | Omnidirectional perception; addresses the planar-LiDAR blind spot |
