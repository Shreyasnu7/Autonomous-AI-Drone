# File: radxa_agent/motion_planner.py
# Converts a validated intent (from IntentValidator) into a smooth,
# FC-safe motion plan (list of position or velocity segments).
#
# Does NOT send MAVLink. Only produces data structures.

import math
import time

class MotionPlanner:

    def __init__(self):
        pass

    # -------------------------------------------------------------
    # MAIN ENTRY POINT
    # -------------------------------------------------------------
    def generate_plan(self, validated_intent: dict):
        """
        Input: SAFE validated intent (from IntentValidator)
        Output: list of motion segments, already smoothed,
                ready for an FC interface to consume.
        """
        plan = []

        motion = validated_intent["motion_profile"]
        sequence = validated_intent["shot_sequence"]
        rel_pos = validated_intent["relative_positioning"]

        max_speed = motion["max_speed_mps"]
        max_accel = motion["max_accel_mps2"]

        # Base offset (initial composition)
        base_forward = rel_pos["offset"]["forward_m"]
        base_right = rel_pos["offset"]["right_m"]
        base_up = rel_pos["offset"]["up_m"]

        for seg in sequence:

            duration = seg["duration_s"]

            # Per-segment offsets added to baseline
            off = seg["target_offset"]

            target_forward = base_forward + off["forward_m"]
            target_right   = base_right + off["right_m"]
            target_up      = base_up + off["up_m"]

            # Flight controller expects structured steps (waypoints or velocity commands)
            segments = self._generate_segment(
                target_forward,
                target_right,
                target_up,
                duration,
                max_speed,
                max_accel,
                camera_adjust=seg["camera_adjust"]
            )

            plan.extend(segments)

        return plan

    # -------------------------------------------------------------
    # GENERATE MOTION FOR A SINGLE SEGMENT
    # -------------------------------------------------------------
    def _generate_segment(self, fwd, right, up, duration, max_speed, max_accel, camera_adjust):
        """
        Produces a list of smooth steps representing this segment.
        Returns a list of dicts: {vx, vy, vz, yaw_cmd?, hold?}
        """

        steps = []
        dt = 0.2    # size of each FC-friendly chunk (5 Hz)

        # Compute needed velocities from offsets
        vx = fwd / duration
        vy = right / duration
        vz = up / duration

        # Clamp to speed limits
        speed = math.sqrt(vx*vx + vy*vy + vz*vz)
        if speed > max_speed:
            scale = max_speed / speed
            vx *= scale
            vy *= scale
            vz *= scale

        # Camera adjustments (FC interface will apply yaw/gimbal later)
        yaw_follow = camera_adjust.get("yaw_to_subject", True)
        pitch_deg  = camera_adjust.get("pitch_deg", -10)

        t = 0
        while t < duration:
            steps.append({
                "vx": vx,       # desired forward speed (m/s)
                "vy": vy,       # desired right speed (m/s)
                "vz": vz,       # desired upward speed (m/s)
                "yaw_follow_subject": yaw_follow,
                "pitch_deg": pitch_deg,
                "dt": dt        # step duration
            })
            t += dt

        return steps
