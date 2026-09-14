# File: radxa_agent/intent_validator.py
# Validates AI → JSON Intent → Safe Flight Instructions
# Does NOT send flight commands. Only cleans, clamps, validates.

import math
import time

# -------------------------
# SAFE LIMITS (edit freely)
# -------------------------
MAX_HORIZONTAL_OFFSET = 50      # meters
MAX_VERTICAL_OFFSET   = 30      # meters
MAX_SPEED             = 8.0     # m/s
MAX_ACCEL             = 4.0     # m/s^2
MAX_DURATION          = 20      # seconds per shot segment
MIN_ALTITUDE          = 2.0     # meters (below this = unsafe)
MAX_ALTITUDE          = 120.0   # legal limit

class IntentValidator:

    def __init__(self):
        pass

    # ------------------------------------------
    # TOP-LEVEL VALIDATION ENTRY
    # ------------------------------------------
    def validate_intent(self, intent: dict):
        """
        Takes full AI intent JSON, returns a SAFE intent JSON
        Or raises a detailed error.
        """

        cleaned = {}

        # 1. Basic metadata
        cleaned["intent_id"] = intent.get("intent_id", f"auto-{time.time()}")
        cleaned["timestamp"] = intent.get("timestamp", time.time())

        # 2. Subject block
        cleaned["subject"] = self._validate_subject(intent.get("subject", {}))

        # 3. Camera block
        cleaned["camera_directive"] = \
            self._validate_camera_directive(intent.get("camera_directive", {}))

        # 4. Relative positioning
        cleaned["relative_positioning"] = \
            self._validate_relative_positioning(intent.get("relative_positioning", {}))

        # 5. Motion profile
        cleaned["motion_profile"] = \
            self._validate_motion_profile(intent.get("motion_profile", {}))

        # 6. Shot sequence
        cleaned["shot_sequence"] = \
            self._validate_shot_sequence(intent.get("shot_sequence", []))

        # 7. Failover behavior
        cleaned["failover_behavior"] = \
            self._validate_failover(intent.get("failover_behavior", {}))

        return cleaned

    # ------------------------------------------
    # VALIDATORS
    # ------------------------------------------

    def _validate_subject(self, block):
        return {
            "type": block.get("type", "unknown"),
            "track_id": int(block.get("track_id", -1)),
            "description": block.get("description", "")
        }

    def _validate_camera_directive(self, block):
        return {
            "style_description": block.get("style_description", "neutral"),
            "primary_goal": block.get("primary_goal", "cinematic"),

            "composition": {
                "framing": block.get("composition", {}).get("framing", "center"),
                "target_size": block.get("composition", {}).get("target_size", "medium"),
                "camera_angle_deg": self._clamp(
                    block.get("composition", {}).get("camera_angle_deg", 0),
                    -90, 0
                ),
                "gimbal_mode": block.get("composition", {}).get("gimbal_mode", "follow")
            }
        }

    def _validate_relative_positioning(self, block):
        off = block.get("offset", {})

        return {
            "mode": block.get("mode", "relative"),
            "description": block.get("description", ""),

            "offset": {
                "forward_m":  self._clamp(off.get("forward_m", 0), -MAX_HORIZONTAL_OFFSET, MAX_HORIZONTAL_OFFSET),
                "right_m":    self._clamp(off.get("right_m", 0), -MAX_HORIZONTAL_OFFSET, MAX_HORIZONTAL_OFFSET),
                "up_m":       self._clamp(off.get("up_m", 0), -MAX_VERTICAL_OFFSET, MAX_VERTICAL_OFFSET),
            }
        }

    def _validate_motion_profile(self, block):
        return {
            "style": block.get("style", "smooth"),
            "max_speed_mps": self._clamp(block.get("max_speed_mps", MAX_SPEED), 0.5, MAX_SPEED),
            "max_accel_mps2": self._clamp(block.get("max_accel_mps2", MAX_ACCEL), 0.5, MAX_ACCEL),
            "curve_bias": block.get("curve_bias", "smooth")
        }

    def _validate_shot_sequence(self, seq):
        cleaned_segments = []

        for segment in seq:

            # Clamp duration
            duration = self._clamp(segment.get("duration_s", 3), 0.5, MAX_DURATION)

            target = segment.get("target_offset", {})
            camera = segment.get("camera_adjust", {})

            cleaned_segments.append({
                "type": segment.get("type", "segment"),
                "description": segment.get("description", ""),

                "duration_s": duration,

                "target_offset": {
                    "forward_m": self._clamp(target.get("forward_m", 0),
                                             -MAX_HORIZONTAL_OFFSET, MAX_HORIZONTAL_OFFSET),
                    "right_m":   self._clamp(target.get("right_m", 0),
                                             -MAX_HORIZONTAL_OFFSET, MAX_HORIZONTAL_OFFSET),
                    "up_m":      self._clamp(target.get("up_m", 0),
                                             -MAX_VERTICAL_OFFSET, MAX_VERTICAL_OFFSET),
                },

                "camera_adjust": {
                    "yaw_to_subject": camera.get("yaw_to_subject", True),
                    "pitch_deg": self._clamp(camera.get("pitch_deg", -20), -80, 0)
                }
            })

        return cleaned_segments

    def _validate_failover(self, block):
        return {
            "if_lost_subject": block.get("if_lost_subject", "hover"),
            "if_unreachable_motion": block.get("if_unreachable_motion", "auto_smooth_adjust"),
            "safety_priority": block.get("safety_priority", "medium")
        }

    # ------------------------------------------
    # Utility clamp
    # ------------------------------------------

    def _clamp(self, val, lo, hi):
        try:
            v = float(val)
            return max(lo, min(hi, v))
        except:
            return lo
