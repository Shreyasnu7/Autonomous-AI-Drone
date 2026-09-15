# Test Suite

Validation harnesses for the Autonomous AI Drone. See [../docs/VERIFICATION.md](../docs/VERIFICATION.md) for methodology and full results.

---

## Available harnesses

| File | Layer | Checks | Requires |
|---|---|---|---|
| `sitl_bridge_test.py` | Firmware-in-the-loop | 13 | ArduCopter SITL binary |
| `sitl_mission_test.py` | Firmware-in-the-loop | 10 | ArduCopter SITL binary |
| `test_depth_projection.py` | Camera depth geometry + object size | 10 | nothing (pure numpy) |
| `test_dome.py` | Spherical memory + active looking | 16 | nothing (pure python) |

Both bind **production bridge methods** to real ArduPilot firmware. They are not mocks of the bridge — a failure here is a failure that would occur in flight.

---

## Setup

Download the SITL binary and default parameters once:

```bash
mkdir -p ~/sitl && cd ~/sitl
curl -sL -o arducopter \
  https://firmware.ardupilot.org/Copter/stable/SITL_x86_64_linux_gnu/arducopter
chmod +x arducopter
curl -sL -o copter.parm \
  https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/autotest/default_params/copter.parm
```

## Running

```bash
python3 tests/sitl_bridge_test.py       # 13 checks: arm, modes, takeoff, velocity, RTL, land, no-GPS
python3 tests/sitl_mission_test.py      # 10 checks: map-route mission upload and execution
python3 tests/test_depth_projection.py  # 10 checks: camera depth geometry + object size (no hardware)
python3 tests/test_dome.py              # 16 checks: dome memory + active looking (no hardware)
```

Expected output ends with:

```
SITL RESULT: 13 passed / 0 failed  (REAL ArduPilot firmware in the loop)
ROUTE MISSION RESULT: 10 passed / 0 failed
```

---

## Environment notes

These are not incidental — each cost real debugging time.

**Run in the foreground, in a single shell session.** Detached background processes are terminated when the launching shell exits, producing empty output files that look like silent failures.

**Force UTF-8 on Windows hosts.** The bridge module contains non-ASCII log characters; import fails under `cp1252`:
```bash
PYTHONUTF8=1 PYTHONIOENCODING=utf-8 python3 tests/sitl_bridge_test.py
```

**Clean up stale SITL instances with a character class:**
```bash
pkill -f "[a]rducopter"      # correct
pkill -f arducopter          # WRONG — also matches its own launcher shell and kills the test
```

**Allow for EKF warm-up.** SITL needs roughly 30–60 s to acquire a fix and set the EKF origin. The harnesses gate on the firmware's own status events (`origin set`, then `EKF3 using GPS`) rather than fixed delays, because a fixed delay races GPS adoption and produces spurious `requires position` failures.

---

## Harnesses pending restoration

The following suites were executed and passed during development, but their source was lost when ephemeral working directories were pruned before being committed — the exact failure mode documented as lesson 5 in the [build journal](../docs/BUILD_JOURNAL.md). They are listed here for transparency and to be rewritten.

| Suite | Layer | Checks | Last result |
|---|---|---|---|
| `test_local_command_path.py` | Command path vs. recording stub FC | 37 | ✅ 37/37 |
| `verify_links.py` | Wire-shape verification across all interfaces | 44 | ✅ 44/44 |
| `test_anchor_memory.py` | Depth-anchor and obstacle-memory algorithms | 11 | ✅ 11/11 |

Their coverage is documented in [../docs/VERIFICATION.md](../docs/VERIFICATION.md) §3, §4 and is largely subsumed by the SITL harnesses, which test the same command paths against real firmware rather than a stub.

---

## Adding tests

1. Bind **production** methods — never reimplement the logic under test.
2. Assert against the **firmware's reported state**, not against what was sent.
3. Gate on events, not timeouts, wherever the system emits a readiness signal.
4. Commit the harness in the same change as the fix it validates.
