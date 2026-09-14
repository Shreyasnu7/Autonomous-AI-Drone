# radxa_agent/plan_executor.py
import json, time, os
from validator import validate_primitive
from config import LAST_EXECUTED_PATH

def simulate_execution(safe_plan: dict, telemetry_info: dict):
    """
    Simulate execution by writing the plan and a synthetic telemetry event.
    This function is intentionally non-destructive and contains no motor outputs.
    Later you can replace the contents of execute_on_fc() to call MAVSDK commands.
    """
    exec_record = {
        "timestamp": time.time(),
        "plan": safe_plan,
        "telemetry_snapshot": telemetry_info
    }
    with open(LAST_EXECUTED_PATH, "w") as f:
        json.dump(exec_record, f, indent=2)
    # Append to a simple log for traceability
    with open("/tmp/radxa_exec_log.txt", "a") as lf:
        lf.write(json.dumps(exec_record) + "\n")

def execute_on_fc_placeholder(safe_plan: dict):
    """
    PLACEHOLDER: This does NOT actuate motors.
    When you are ready (after SITL + verification + operator controls),
    replace this function with MAVSDK or the system that issues commands
    to the autopilot.

    Example comments for future implementation:
      1. Connect to MAVSDK (System), verify .telemetry.armed() is False or True as required
      2. Use drone.action or drone.offboard.set_velocity_body() carefully (after preflight)
      3. Ensure offboard setpoint heartbeat is running and failsafe is configured
      4. Always keep an "abort" channel listening for RC takeover

    DO NOT enable automatic actuation until you've fully validated.
    """
    # Instead of sending to FC, we simulate/record only
    simulate_execution(safe_plan, {"simulated": True})
