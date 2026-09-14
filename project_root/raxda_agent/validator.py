# radxa_agent/validator.py
from config import (MAX_RADIUS_M, MIN_RADIUS_M, MAX_ALT_M, MIN_ALT_M,
                    MAX_SPEED_MS, MAX_YAW_DEG_S)

def clamp(val, lo, hi):
    try:
        v = float(val)
    except Exception:
        return lo
    if v < lo: return lo
    if v > hi: return hi
    return v

def validate_primitive(prim: dict) -> dict:
    """
    Input: primitive dictionary created by laptop AI
    Output: sanitized primitive with safety metadata
    """
    action = prim.get("action", "").upper()
    params = prim.get("params", {}) or {}
    safe = {"action": action, "params": {}, "meta": {"raw": prim}}

    if action == "FOLLOW":
        d = clamp(params.get("distance_m", 8), MIN_RADIUS_M, MAX_RADIUS_M)
        h = clamp(params.get("height_m", 4), MIN_ALT_M, MAX_ALT_M)
        safe["params"]["distance_m"] = d
        safe["params"]["height_m"] = h
        safe["params"]["speed_ms"] = clamp(params.get("speed_ms", 2.0), 0.2, MAX_SPEED_MS)

    elif action == "ORBIT":
        r = clamp(params.get("radius_m", 10), MIN_RADIUS_M, MAX_RADIUS_M)
        speed = clamp(params.get("speed_ms", 2.0), 0.2, MAX_SPEED_MS)
        safe["params"]["radius_m"] = r
        safe["params"]["speed_ms"] = speed

    elif action == "PULLBACK":
        d = clamp(params.get("distance_m", 12), 2, 200)
        safe["params"]["distance_m"] = d

    elif action == "REVEAL":
        h = clamp(params.get("height_change", 5), 1, 50)
        safe["params"]["height_change"] = h

    elif action == "HOVER":
        safe["params"] = {}

    else:
        # Unknown action, convert to HOVER as safe fallback
        safe["action"] = "HOVER"
        safe["params"] = {}

    # add general safe caps
    safe["params"]["max_speed_ms"] = clamp(params.get("max_speed_ms", MAX_SPEED_MS), 0.2, MAX_SPEED_MS)
    safe["params"]["max_yaw_deg_s"] = clamp(params.get("max_yaw_deg_s", MAX_YAW_DEG_S), -MAX_YAW_DEG_S, MAX_YAW_DEG_S)
    return safe
