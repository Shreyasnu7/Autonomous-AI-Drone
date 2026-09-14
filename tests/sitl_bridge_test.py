"""SITL BRIDGE VALIDATION — the production bridge vs. REAL ArduPilot firmware (no hardware).

Launches ArduCopter SITL (the same firmware family that runs on the aircraft), binds the REAL
real_bridge_service methods to a live MAVLink connection, then drives the EXACT command paths the
ground station and application use, asserting the firmware's actual response (mode, armed state,
altitude, ground speed, parameter values).

This is the strongest pre-hardware validation available: faults found here are faults that would
have occurred in flight.

Result at time of writing: 13/13.

Prerequisites
-------------
  ArduCopter SITL binary + default parameters at ~/sitl/
      curl -sL -o ~/sitl/arducopter \\
        https://firmware.ardupilot.org/Copter/stable/SITL_x86_64_linux_gnu/arducopter
      chmod +x ~/sitl/arducopter
      curl -sL -o ~/sitl/copter.parm \\
        https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/autotest/default_params/copter.parm

Usage
-----
  python3 tests/sitl_bridge_test.py

Notes
-----
  * Run in the FOREGROUND in a single shell session. Detached processes are terminated when the
    launching shell exits.
  * On Windows hosts set PYTHONUTF8=1 — the bridge module contains non-ASCII log characters.
  * Use `pkill -f "[a]rducopter"` (character class) to clean up stale instances; a bare
    `pkill -f arducopter` matches its own launcher shell and kills the test too.
"""
import os
import sys
import time
import math
import types
import asyncio
import inspect
import importlib.util
import subprocess
import threading

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "raxda_bridge"))
from pymavlink import mavutil  # noqa: E402

# --------------------------------------------------------------------------------------
# Launch real ArduPilot firmware
# --------------------------------------------------------------------------------------
SITL_HOME = os.path.expanduser("~/sitl")
sitl = subprocess.Popen(
    [f"{SITL_HOME}/arducopter", "--model", "quad", "-w",
     "--defaults", f"{SITL_HOME}/copter.parm",
     "--home", "-35.363262,149.165237,584,0"],
    cwd=SITL_HOME, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
print("SITL launched, connecting tcp:127.0.0.1:5760 ...", flush=True)
time.sleep(3)

fc = mavutil.mavlink_connection("tcp:127.0.0.1:5760")
fc.wait_heartbeat(timeout=30)
print(f"HEARTBEAT from sysid {fc.target_system} (REAL ArduPilot firmware)", flush=True)

# --------------------------------------------------------------------------------------
# Bind the PRODUCTION bridge methods to the live firmware connection
# --------------------------------------------------------------------------------------
spec = importlib.util.spec_from_file_location(
    "rbs", os.path.join(REPO, "raxda_bridge", "real_bridge_service.py"))
rbs = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rbs)
BridgeCls = next(o for _, o in vars(rbs).items()
                 if inspect.isclass(o) and hasattr(o, "_execute_named_command"))

bridge = types.SimpleNamespace()
bridge.fc = fc
bridge.safety = types.SimpleNamespace(validate_auto_action=lambda a: True)
bridge.telemetry_cache = {"gps_fix": 0, "altitude": 0.0, "mode_id": "STABILIZE", "yaw": 0.0,
                          "t1": -1, "t2": -1, "t3": -1, "t4": -1}
bridge.batt_rth_destination = "launch"
bridge.user_gps = None
bridge.obstacle_avoidance = None
bridge.esp32_cmd_queue = None
bridge.running = True
bridge.is_armed = False
bridge.local_clients = set()
for m in ("process_packet", "execute_ai_plan", "_execute_named_command",
          "_land_backstop_disarm", "_goto_then_land", "_smart_avoidance_monitor"):
    if hasattr(BridgeCls, m):
        setattr(bridge, m, types.MethodType(getattr(BridgeCls, m), bridge))

# --------------------------------------------------------------------------------------
# Telemetry pump — mirrors what the real bridge read-loop does
# --------------------------------------------------------------------------------------
MODES = {0: "STABILIZE", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
         5: "LOITER", 6: "RTL", 9: "LAND", 16: "POSHOLD"}

state = {"mode": None, "armed": False, "alt": 0.0, "fix": 0, "gspd": 0.0,
         "rc3": 0, "wpnav": None, "origin": False, "gps_pos": False}


def pump():
    while True:
        msg = fc.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue
        t = msg.get_type()
        if t == "HEARTBEAT":
            state["mode"] = msg.custom_mode
            state["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            bridge.is_armed = state["armed"]
            bridge.telemetry_cache["mode_id"] = MODES.get(msg.custom_mode, str(msg.custom_mode))
        elif t == "GLOBAL_POSITION_INT":
            state["alt"] = msg.relative_alt / 1000.0
            state["gspd"] = math.hypot(msg.vx, msg.vy) / 100.0
        elif t == "GPS_RAW_INT":
            state["fix"] = msg.fix_type
        elif t == "RC_CHANNELS":
            state["rc3"] = msg.chan3_raw
        elif t == "COMMAND_ACK":
            print(f"    [ack] cmd {msg.command} result {msg.result}", flush=True)
        elif t == "STATUSTEXT":
            print(f"    [fc] {msg.text}", flush=True)
            # The EKF readiness sequence: 'origin set' precedes GPS adoption for lateral position.
            # GUIDED is refused with 'requires position' until the LATTER occurs.
            if "origin set" in msg.text:
                state["origin"] = True
            if "using GPS" in msg.text:
                state["gps_pos"] = True
        elif t == "PARAM_VALUE":
            pid = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode("utf-8", "ignore")
            if pid.strip("\x00 ") == "WPNAV_SPEED":
                state["wpnav"] = float(msg.param_value)
        bridge.telemetry_cache["gps_fix"] = state["fix"]
        bridge.telemetry_cache["altitude"] = state["alt"]


threading.Thread(target=pump, daemon=True).start()

ok = bad = 0


def chk(name, cond, detail=""):
    global ok, bad
    print(f"  [{'PASS' if cond else 'FAIL'}] {name} {detail}", flush=True)
    ok, bad = ok + (1 if cond else 0), bad + (0 if cond else 1)


def wait_for(predicate, timeout, poll=0.3):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if predicate():
            return True
        time.sleep(poll)
    return False


# --------------------------------------------------------------------------------------
async def main():
    fc.mav.request_data_stream_send(fc.target_system, fc.target_component,
                                    mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
    print("waiting for EKF/GPS (SITL warm-up ~30 s) ...", flush=True)
    chk("SITL GPS 3D fix", wait_for(lambda: state["fix"] >= 3, 90), f"fix={state['fix']}")

    fc.mav.param_set_send(fc.target_system, fc.target_component, b"ARMING_CHECK", 0,
                          mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    # The production bridge sets DISARM_DELAY=0 on connect; replicate it here because this
    # harness bypasses the bridge's connect-time parameter initialisation.
    fc.mav.param_set_send(fc.target_system, fc.target_component, b"DISARM_DELAY", 0,
                          mavutil.mavlink.MAV_PARAM_TYPE_INT8)

    # Gate on EKF EVENTS rather than fixed sleeps — a fixed delay races GPS adoption.
    chk("EKF origin set (position estimate ready)",
        wait_for(lambda: state["origin"], 120), f"origin={state['origin']}")
    wait_for(lambda: state["gps_pos"], 60)
    time.sleep(5)

    # 1) mode command through the bridge -> real firmware mode
    await bridge._execute_named_command("GUIDED", {})
    chk("bridge GUIDED -> firmware mode GUIDED", wait_for(lambda: state["mode"] == 4, 8),
        f"mode={MODES.get(state['mode'], state['mode'])}")

    # 2) SET_SPEED -> real parameter write on the firmware
    await bridge._execute_named_command("SET_SPEED", {"value": 3.5})
    fc.mav.param_request_read_send(fc.target_system, fc.target_component, b"WPNAV_SPEED", -1)
    chk("bridge SET_SPEED 3.5 m/s -> WPNAV_SPEED=350",
        wait_for(lambda: state["wpnav"] == 350.0, 8), f"wpnav={state['wpnav']}")

    # 3) ARM through the bridge (the restored split-arm sequence)
    armed = False
    for _ in range(6):
        await bridge._execute_named_command("ARM", {})
        armed = wait_for(lambda: state["armed"], 10)
        if armed:
            break
    chk("bridge ARM -> firmware ARMED", armed, f"armed={state['armed']}")

    # 4) TAKEOFF — the bridge ensures GUIDED itself before NAV_TAKEOFF.
    #    (Without that, the firmware refuses with result=4 after a STABILIZE split-arm.)
    if not state["armed"]:
        await bridge._execute_named_command("ARM", {})
        wait_for(lambda: state["armed"], 15)
    await bridge._execute_named_command("TAKEOFF", {"alt": 10})
    chk("bridge TAKEOFF -> climbs past 5 m", wait_for(lambda: state["alt"] > 5.0, 45),
        f"alt={state['alt']:.1f}m")
    wait_for(lambda: state["alt"] > 9.0, 25)      # let the GUIDED takeoff complete
    time.sleep(2)

    # 5) AI velocity path: cmd_vel with a fix -> GUIDED setpoint -> the aircraft actually MOVES
    await bridge._execute_named_command("GUIDED", {})
    wait_for(lambda: state["mode"] == 4, 5)
    for _ in range(40):
        await bridge.process_packet("cmd_vel", {"vx": 3.0, "vy": 0.0, "vz": 0.0, "yaw_rate": 0.0})
        await asyncio.sleep(0.2)
    chk("bridge cmd_vel 3 m/s -> real ground speed > 1.2 m/s", state["gspd"] > 1.2,
        f"gspd={state['gspd']:.2f}m/s")

    # 6) LOITER hold
    await bridge._execute_named_command("LOITER", {})
    chk("bridge LOITER -> firmware LOITER", wait_for(lambda: state["mode"] == 5, 8),
        f"mode={MODES.get(state['mode'], state['mode'])}")

    # 7) GPS-DENIED path, tested while armed and airborne
    fc.mav.param_set_send(fc.target_system, fc.target_component, b"SIM_GPS_DISABLE", 1,
                          mavutil.mavlink.MAV_PARAM_TYPE_INT8)
    fc.mav.param_set_send(fc.target_system, fc.target_component, b"FS_EKF_ACTION", 0,
                          mavutil.mavlink.MAV_PARAM_TYPE_INT8)
    chk("GPS disabled (fix<3)", wait_for(lambda: state["fix"] < 3, 30), f"fix={state['fix']}")
    bridge.telemetry_cache["mode_id"] = "LOITER"
    for _ in range(14):
        await bridge.process_packet("cmd_vel", {"vx": 0.0, "vy": 0.0, "vz": -0.4, "yaw_rate": 0.0})
        await asyncio.sleep(0.2)
    chk("no-GPS cmd_vel -> firmware ALT_HOLD", wait_for(lambda: state["mode"] == 2, 8),
        f"mode={MODES.get(state['mode'], state['mode'])}")
    chk("no-GPS cmd_vel -> RC3 override ~1600 reaches firmware",
        1560 <= state["rc3"] <= 1640, f"rc3={state['rc3']}")

    # 8) RTL and landing
    fc.mav.param_set_send(fc.target_system, fc.target_component, b"SIM_GPS_DISABLE", 0,
                          mavutil.mavlink.MAV_PARAM_TYPE_INT8)
    wait_for(lambda: state["fix"] >= 3, 30)
    await bridge._execute_named_command("RTL", {})
    chk("bridge RTL -> firmware RTL", wait_for(lambda: state["mode"] == 6, 8),
        f"mode={MODES.get(state['mode'], state['mode'])}")
    chk("firmware lands + auto-disarms", wait_for(lambda: not state["armed"], 120),
        f"alt={state['alt']:.1f}m")

    print(f"\nSITL RESULT: {ok} passed / {bad} failed  (REAL ArduPilot firmware in the loop)")
    return 1 if bad else 0


code = asyncio.run(main())
sitl.terminate()
sys.exit(code)
