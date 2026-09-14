# Deployment and Operation

Provisioning the companion computer, deploying the bridge, and the correct start-up order.

> This document supersedes the earlier `DEPLOY_MANUAL.md`, `HOW_TO_RUN_EVERYTHING.md` and
> `RADXA_DEPLOY_PACKAGE.md`, which referenced a previous board revision, an SD-card path that is no
> longer mounted, a cloud relay that is no longer part of the control path, and an obsolete bridge
> revision pasted inline. All values below reflect the current system.

---

## 1. Topology

```
Operator phone ──┐
                 ├── Tailscale mesh ── Companion computer ── UART ── Flight controller
Ground station ──┘                            │
                                              └── USB ── LiDAR,  (camera on its own power)
```

**The control path is entirely local.** Commands travel operator → bridge and ground station → bridge over the Tailscale mesh on the local hotspot. No cloud service sits in the control loop. The only external call is the strategic model, which runs once per objective on the ground station and cannot stall the flight loop.

---

## 2. Companion computer provisioning

Required once per board. The SD card carries the entire stack, so a board swap needs only a card transfer (same model — a different SoC means an incompatible kernel and device tree).

### 2.1 Serial port access

```bash
sudo usermod -a -G dialout $USER
sudo reboot          # required — group membership is applied at login
```

Without this the bridge cannot open the flight-controller UART.

### 2.2 Flight-controller UART

Companion pins **8 and 10** map to `/dev/ttyAS0` at 57600 baud. Enable the corresponding device-tree overlay and reboot.

⚠️ Pin 12 is **not** a UART on the Cubie A7Z. Earlier documentation specified Zero 3W pin mappings — same physical connector, different SoC, different mapping.

### 2.3 Service installation

```bash
mkdir -p ~/raxda_bridge
sudo cp deploy/drone-bridge.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now drone-bridge
```

The unit **must** contain:

```ini
Environment=PYTHONUNBUFFERED=1
```

Without it, Python block-buffers stdout under systemd and the LiDAR loop produces no journal output at all — which reads as a hardware failure. The hardware was healthy; only the logging was invisible. This cost a full debugging session.

### 2.4 Read-only root filesystem (recommended before flight)

Brownouts corrupt SD cards far more often than they damage silicon, and card corruption is one of the two states that make a healthy board appear dead. An overlay root makes the filesystem immune to power loss.

```bash
sudo apt-get install -y overlayroot
sudo nano /etc/overlayroot.conf     # set: overlayroot="tmpfs"
sudo reboot
```

**Enable this only once the configuration is finalised** — with the overlay active, changes live in RAM and are lost on reboot. To deploy afterwards, either disable the overlay temporarily or use `overlayroot-chroot` to write through to the underlying disk.

---

## 3. Deploying the bridge

Deployment is a file copy and a service restart. Nothing is flashed; the companion boots from its SD card, which already carries the stack.

```bash
# From the repository root
scp raxda_bridge/real_bridge_service.py raxda_bridge/lidar_pose.py \
    shreyash@<companion-ip>:~/raxda_bridge/

ssh shreyash@<companion-ip> \
    "python3 -m py_compile ~/raxda_bridge/real_bridge_service.py && \
     sudo systemctl restart drone-bridge && \
     sleep 5 && systemctl is-active drone-bridge"
```

Compiling before restarting catches syntax errors while the previous version is still running.

⚠️ **Always edit locally and copy up.** Editing on the device via inline SSH one-liners caused repeated indentation errors and crash loops. More importantly, on-device edits that never reach version control **will eventually be destroyed** — roughly two weeks of fixes were lost exactly this way (see [BUILD_JOURNAL.md](BUILD_JOURNAL.md) Phase 12).

### 3.1 Back up before overwriting

```bash
ssh shreyash@<companion-ip> \
    "cp ~/raxda_bridge/real_bridge_service.py \
        ~/raxda_bridge/real_bridge_service.py.pre_deploy_\$(date +%s)"
```

---

## 4. Start-up order

Start in this order. Each stage depends on the one before it.

### Stage 1 — Companion bridge (on the aircraft)

Power the aircraft and confirm the companion joins the network.

```bash
ssh shreyash@<companion-ip> "sudo journalctl -u drone-bridge -f"
```

Wait for these signatures:

| Signature | Meaning |
|---|---|
| `✅ FC Connected @ 57600! Heartbeat receiving` | Flight-controller link established |
| `✅ LIDAR SERIAL: Running via raw protocol parser` | LiDAR streaming |
| `📡 LIDAR RING #n: x/72 sectors, fwd=…cm` | Obstacle ring reaching the flight controller |
| `⚙️ fc_caps ANGLE_MAX = …` | Live aircraft envelope read from the FC |
| `🛰️ GPS STATUS: fix=… sats=…` | GNSS state (outdoor) |

A `🔴 FC NOT DETECTED. Retrying...` loop while the board is on a bench supply is expected — it means the bridge is healthy and waiting for the flight controller.

### Stage 2 — Inference server (ground station)

```bash
bash scripts/start_vllm_pilot.sh
```

Wait for `Application startup complete`, then confirm the model is served:

```bash
curl -s http://<host>:8100/v1/models
```

⚠️ **Verify reachability from the machine that will run the director.** Under WSL2 mirrored networking, the Windows host cannot reach a server inside WSL by default — the director then silently falls back to a slower in-process engine. If `curl` from the host fails, add to `~/.wslconfig`:

```ini
[experimental]
hostAddressLoopback=true
```

then `wsl --shutdown` and restart the server.

### Stage 3 — Director (ground station)

```bash
cd ai/camera_brain && python -m laptop_ai.director_core
```

| Signature | Meaning |
|---|---|
| `⚡ Detector: YOLO-World TensorRT (…engine, classes baked)` | Accelerated detector loaded |
| `✅ Pilot using FAST vLLM engine @ …` | Pilot connected to the inference server |
| `CONNECTING TO VPS_WS = ws://<ip>:8000` then `MessagingClient connected` | Bridge link established |

⚠️ `⚠️ FAST engine offline … Falling back to slow in-process Qwen` means Stage 2 is unreachable. Fix the link rather than proceeding — the fallback engine will not meet the loop-latency target.

### Stage 4 — Operator application

1. Open the application; set connection mode to **tailscale** and enter the companion's current Tailscale address.
2. Confirm telemetry updates, then video.

⚠️ The companion's Tailscale address **changes when the node is re-registered** (for example after a board replacement). A stale address in the application is the most common cause of "no telemetry". Confirm the current address with `tailscale status` before assuming a fault.

---

## 5. Operator notes

**Proximity stop is expected behaviour.** With an obstacle inside roughly 50 cm, the aircraft refuses AI movement commands and holds. Journal output reads `🛑 Pi0 BRAKE: obs 29cm < safe 45cm`. This is the safety layer working, not a fault. On a bench the aircraft is usually surrounded by clutter and will report braking continuously.

**Route missions.** Tap waypoints on the map, press START to upload, then press ARM. The bridge uploads via the MAVLink mission handshake, prepends an automatic takeoff item, and switches to AUTO once the firmware acknowledges. The application shows `MISSION UPLOADED (n waypoints)`. Requires `AUTO_OPTIONS=3` on the flight controller — see [HARDWARE.md §3.2](HARDWARE.md).

**Return policy** — destination and battery threshold are set in the application and honoured by every return path, manual and automatic. Without a position fix, home and operator returns degrade to land-in-place rather than attempting a blind return.

---

## 6. Emergency procedures

Full procedures in [../SAFETY.md](../SAFETY.md). Immediate actions:

| Situation | Action |
|---|---|
| Uncommanded movement | Switch to a manual mode on the transmitter |
| Stop autonomy immediately | `sudo systemctl stop drone-bridge` — the flight controller keeps flying, AI control ends |
| Land now | LAND in the application, or transmitter mode switch |
| Imminent collision with a person | **Disarm.** A crashed aircraft is preferable to an injury |

The transmitter is always authoritative. RC overrides expire after 3 seconds, so if the ground station stops the sticks neutralise automatically.

---

## 7. Troubleshooting

| Symptom | Likely cause | Check |
|---|---|---|
| No telemetry in the application | Stale Tailscale address | `tailscale status`; update the application |
| No telemetry, address correct | Companion offline | Hotspot on and 2.4 GHz? Companion powered? |
| Bridge active, no LiDAR lines | stdout buffering | `Environment=PYTHONUNBUFFERED=1` present in the unit? |
| `FC NOT DETECTED` loop | UART not owned or not wired | `dialout` group; overlay enabled; wiring |
| Pilot slow, loop > 1 s | Inference server unreachable | `curl <host>:8100/v1/models` from the director's machine |
| Detector classes wrong | Engine vocabulary is stale | Re-export: `python scripts/build_trt_world.py` |
| Arms but will not take off | Not in GUIDED, or EKF not ready | Wait for `EKF3 using GPS`, not merely a 3D fix |
| Mission uploads, never flies | `AUTO_OPTIONS` unset | Set to 3 — see [HARDWARE.md §3.2](HARDWARE.md) |
| Companion died mid-session | **Power integrity** | [HARDWARE.md §4](HARDWARE.md) — five boards were lost to this |

⚠️ The bridge owns the flight-controller serial port exclusively. Stop the service before running any direct MAVLink tooling, and restart it afterwards.
