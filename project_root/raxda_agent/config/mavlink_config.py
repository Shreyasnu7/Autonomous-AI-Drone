# mavlink_config.py

MAVLINK_CONNECTION = "udp:127.0.0.1:14550"  
# For real flight: "serial:/dev/ttyAMA0:57600" or telemetry radio

SYSTEM_ID = 1
COMPONENT_ID = 1

SETPOINT_RATE_HZ = 30        # high precision, low latency
TIMEOUT_SECONDS = 1.0        # stop if AI stops sending
FAILSAFE_HOVER_TIME = 3.0    # hover before RTL or disarm
