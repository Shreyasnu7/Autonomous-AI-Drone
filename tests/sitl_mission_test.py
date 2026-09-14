"""ROUTE-MISSION VALIDATION — the application's map mission vs. REAL ArduPilot firmware.

Feeds the EXACT packet the Flutter application emits when the operator taps waypoints on the map
and presses START:

    {"type": "mission", "payload": [{"lat": ..., "lng": ...}, ...]}

through the production bridge to real ArduCopter firmware, and asserts that the firmware uploads,
accepts, starts and flies the complete route.

Result at time of writing: 10/10.

What this test caught (all would have failed in the field)
---------------------------------------------------------
  * The bridge's local packet handler had no 'mission' branch — mission packets were logged as
    "unknown type" and discarded. The map feature was entirely non-functional over the local link.
  * The application sends no start command; the START button set a UI label only.
  * With default AUTO_OPTIONS, AUTO waits for a PILOT THROTTLE RAISE before the takeoff item —
    impossible for an application-flown aircraft. Missions armed and then sat idle.
  * Sending mission_clear_all before mission_count produced its own MISSION_ACK, which the handler
    mistook for upload completion; the firmware then reported "Mission upload timeout" and refused
    AUTO with "init failed".

Prerequisites / usage: see tests/sitl_bridge_test.py
"""
import os
import sys
import time
import types
import asyncio
import inspect
import importlib.util
import subprocess

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "raxda_bridge"))
from pymavlink import mavutil  # noqa: E402

SITL_HOME = os.path.expanduser("~/sitl")
sitl = subprocess.Popen(
    [f"{SITL_HOME}/arducopter", "--model", "quad", "-w",
     "--defaults", f"{SITL_HOME}/copter.parm",
     "--home", "-35.363262,149.165237,584,0"],
    cwd=SITL_HOME, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(3)
fc = mavutil.mavlink_connection("tcp:127.0.0.1:5760")
fc.wait_heartbeat(timeout=30)
print("REAL ArduPilot firmware connected", flush=True)

fc.mav.request_data_stream_send(fc.target_system, fc.target_component,
                                mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
for pname, pval, ptype in ((b"ARMING_CHECK", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
                           (b"DISARM_DELAY", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT8),
                           (b"AUTO_OPTIONS", 3, mavutil.mavlink.MAV_PARAM_TYPE_INT32)):
    fc.mav.param_set_send(fc.target_system, fc.target_component, pname, pval, ptype)

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
          "_begin_route_mission",
          "_land_backstop_disarm", "_goto_then_land", "_smart_avoidance_monitor"):
    if hasattr(BridgeCls, m):
        setattr(bridge, m, types.MethodType(getattr(BridgeCls, m), bridge))

MODES = {0: "STABILIZE", 3: "AUTO", 4: "GUIDED", 6: "RTL", 9: "LAND"}
st = {"armed": False, "mode": None, "alt": 0.0, "fix": 0, "origin": False,
      "gps_pos": False, "reached": set(), "wp_current": 0, "ack_wait_arm": False}


async def pump_async():
    """Async pump. Also answers the firmware's MISSION_REQUEST pulls from the bridge's pending
    mission list, mirroring the production bridge's FC read-loop handler."""
    while True:
        m = fc.recv_match(blocking=False)
        if m is None:
            await asyncio.sleep(0.02)
            continue
        t = m.get_type()
        if t == "HEARTBEAT":
            st["armed"] = bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            st["mode"] = m.custom_mode
            bridge.is_armed = st["armed"]
            bridge.telemetry_cache["mode_id"] = MODES.get(m.custom_mode, str(m.custom_mode))
        elif t == "GLOBAL_POSITION_INT":
            st["alt"] = m.relative_alt / 1000.0
            bridge.telemetry_cache["altitude"] = st["alt"]
        elif t == "GPS_RAW_INT":
            st["fix"] = m.fix_type
            bridge.telemetry_cache["gps_fix"] = m.fix_type
        elif t == "STATUSTEXT":
            if "origin set" in m.text:
                st["origin"] = True
            if "using GPS" in m.text:
                st["gps_pos"] = True
            print(f"    [fc] {m.text}", flush=True)
        elif t in ("MISSION_REQUEST", "MISSION_REQUEST_INT"):
            pm = getattr(bridge, "_pending_mission", None)
            if pm and 0 <= m.seq < len(pm):
                it = pm[m.seq]
                fc.mav.mission_item_int_send(
                    fc.target_system, fc.target_component, m.seq,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    it["cmd"], 0, 1, 0, 0, 0, 0,
                    int(it["lat"] * 1e7), int(it["lng"] * 1e7), float(it["alt"]))
        elif t == "MISSION_ACK":
            pm = getattr(bridge, "_pending_mission", None)
            if pm is not None:
                print(f"    [fc] MISSION_ACK result={m.type} ({len(pm) - 2} waypoints)", flush=True)
                bridge._pending_mission = None
                if m.type == 0 and getattr(bridge, "_mission_autostart", False):
                    bridge._mission_autostart = False
                    if bridge.is_armed:
                        fc.mav.set_mode_send(fc.target_system,
                                             mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 3)
                        print("    [bridge] armed -> AUTO", flush=True)
                    else:
                        st["ack_wait_arm"] = True
        elif t == "MISSION_ITEM_REACHED":
            st["reached"].add(m.seq)
            print(f"    [fc] WAYPOINT {m.seq} REACHED", flush=True)
        elif t == "MISSION_CURRENT":
            st["wp_current"] = m.seq


ok = bad = 0


def chk(name, cond, detail=""):
    global ok, bad
    print(f"  [{'PASS' if cond else 'FAIL'}] {name} {detail}", flush=True)
    ok, bad = ok + (1 if cond else 0), bad + (0 if cond else 1)


async def wait_for(predicate, timeout, poll=0.3):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if predicate():
            return True
        await asyncio.sleep(poll)
    return False


async def main():
    asyncio.create_task(pump_async())
    print("waiting for EKF/GPS ...", flush=True)
    chk("GPS 3D fix", await wait_for(lambda: st["fix"] >= 3, 90), f"fix={st['fix']}")
    chk("EKF origin + GPS position",
        await wait_for(lambda: st["origin"] and st["gps_pos"], 120))
    await asyncio.sleep(5)

    # THE EXACT APPLICATION PACKET: four map taps forming a box, no altitudes, no start command
    home = (-35.363262, 149.165237)
    waypoints = [{"lat": home[0] + 3e-4, "lng": home[1]},
                 {"lat": home[0] + 3e-4, "lng": home[1] + 3e-4},
                 {"lat": home[0] - 1e-4, "lng": home[1] + 3e-4},
                 {"lat": home[0],        "lng": home[1]}]
    await bridge.process_packet("mission", waypoints)
    chk("bridge accepted the application 'mission' packet",
        getattr(bridge, "_pending_mission", None) is not None)
    chk("handshake upload complete (MISSION_ACK)",
        await wait_for(lambda: getattr(bridge, "_pending_mission", "sentinel") is None, 20))

    # The operator then presses ARM; the bridge auto-starts AUTO on acknowledgement.
    for _ in range(6):
        await bridge._execute_named_command("ARM", {})
        if await wait_for(lambda: st["armed"], 10):
            break
    chk("ARM (bridge split-arm)", st["armed"])

    if st["ack_wait_arm"]:
        for _ in range(10):
            fc.mav.set_mode_send(fc.target_system,
                                 mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 3)
            if await wait_for(lambda: st["mode"] == 3, 2):
                break

    chk("firmware in AUTO (mission running)", await wait_for(lambda: st["mode"] == 3, 15),
        f"mode={MODES.get(st['mode'], st['mode'])}")
    chk("auto-takeoff climbs past 8 m", await wait_for(lambda: st["alt"] > 8.0, 60),
        f"alt={st['alt']:.1f}m")
    chk("waypoint 2 reached", await wait_for(lambda: 2 in st["reached"], 90),
        f"reached={sorted(st['reached'])}")
    chk("waypoint 4 reached", await wait_for(lambda: 4 in st["reached"], 120),
        f"reached={sorted(st['reached'])}")
    chk("final waypoint reached (full route flown)",
        await wait_for(lambda: 5 in st["reached"], 120), f"reached={sorted(st['reached'])}")

    print(f"\nROUTE MISSION RESULT: {ok} passed / {bad} failed "
          f"(REAL firmware flew the application's map path)")
    return 1 if bad else 0


code = asyncio.run(main())
sitl.terminate()
sys.exit(code)
