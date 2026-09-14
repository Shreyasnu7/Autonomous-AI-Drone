# radxa_agent/telemetry_sim.py
# Small helper to create simulated telemetry snapshots for logs / server
import time, random

def create_telemetry_snapshot():
    return {
        "time": time.time(),
        "lat": 0.0 + random.uniform(-0.0001, 0.0001),
        "lon": 0.0 + random.uniform(-0.0001, 0.0001),
        "alt_m": 5.0 + random.uniform(-0.5, 0.5),
        "battery": 0.9 + random.uniform(-0.05, 0.0),
        "sats": 10,
        "armed": False
    }
