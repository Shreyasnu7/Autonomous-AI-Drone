"""Active sensing - decide where to POINT the sensors to fill in the dome.

The aircraft carries two steerable sensors and, before this, used neither deliberately:

  * the gimbal, which can aim the camera cone anywhere across a wide arc, and
  * the airframe itself, whose pitch and roll tilt the LiDAR's single horizontal plane through
    elevation.

Both were only ever moved for other reasons - the gimbal to frame a subject, the airframe to fly -
so which parts of the surroundings got measured was incidental. This chooses deliberately: look
where we have NOT looked, prefer the direction we are about to move into, and stop sweeping when
the picture is good enough so the sensors return to their real jobs.

Two principles it will not break:

  * SWEEPING NEVER COMMANDS THE AIRFRAME TO MOVE. It only ever requests a gimbal angle, or a small
    pitch bias while the aircraft is already flying. It cannot start, stop or steer a flight.
  * IT YIELDS. Any real task - a subject to track, an obstacle to avoid, a manoeuvre in progress -
    takes the sensors back immediately, because a scan is never more important than the thing the
    aircraft is actually doing.
"""
import math
import time


class DomeScanner:
    # Gimbal sweep pattern: a handful of yaw stops either side, plus a look up and down. Kept
    # coarse because the camera cone is wide; the point is coverage, not resolution.
    YAW_STOPS_DEG = (-60.0, -30.0, 0.0, 30.0, 60.0)
    PITCH_STOPS_DEG = (-20.0, 0.0, 20.0)
    DWELL_S = 0.6                 # time held at each stop, long enough for a depth frame
    COVERAGE_TARGET = 0.45        # stop sweeping once this much of the dome is known
    RESUME_BELOW = 0.30           # and start again if knowledge decays past this
    TILT_BIAS_DEG = 6.0           # airframe pitch bias used to sweep the LiDAR plane
    TILT_PERIOD_S = 4.0

    def __init__(self):
        self._idx = 0
        self._last_step = 0.0
        self._active = False
        self._suspended_reason = None

    # ---- gimbal ------------------------------------------------------------
    def next_gimbal(self, dome, busy=False, travel_az_deg=None, now=None):
        """Where to point the camera next, or None to leave the gimbal alone.

        Returns (pitch_deg, yaw_deg). None means "not scanning" - the caller keeps whatever
        aiming the real task wants, which is always the higher priority.
        """
        now = now or time.time()
        if busy:
            self._active = False
            self._suspended_reason = "aircraft busy"
            return None

        cov = dome.coverage() if dome is not None else 0.0
        if self._active and cov >= self.COVERAGE_TARGET:
            self._active = False
            self._suspended_reason = "coverage reached"
            return None
        if not self._active:
            if cov > self.RESUME_BELOW:
                return None
            self._active = True
            self._suspended_reason = None
            self._idx = 0
            self._last_step = 0.0

        if now - self._last_step < self.DWELL_S:
            return None                      # still dwelling at the current stop
        self._last_step = now

        # Prefer an unobserved direction, and among those the one nearest where we are heading:
        # knowing what is in front of the flight path is worth more than knowing what is behind.
        stops = list(self.YAW_STOPS_DEG)
        if dome is not None:
            gaps = dome.unobserved_directions(el_deg=0.0)
            if gaps:
                reachable = [g for g in gaps if min(self.YAW_STOPS_DEG) - 15 <= g <= max(self.YAW_STOPS_DEG) + 15]
                if reachable:
                    if travel_az_deg is not None:
                        reachable.sort(key=lambda a: abs(((a - travel_az_deg + 180) % 360) - 180))
                    stops = reachable

        yaw = stops[self._idx % len(stops)]
        pitch = self.PITCH_STOPS_DEG[(self._idx // max(1, len(stops))) % len(self.PITCH_STOPS_DEG)]
        self._idx += 1
        return (float(pitch), float(max(-90.0, min(90.0, yaw))))

    # ---- airframe tilt -----------------------------------------------------
    def tilt_bias(self, dome, airborne=False, busy=False, now=None):
        """A small pitch bias, in degrees, that sweeps the LiDAR plane through elevation.

        Returns 0.0 unless the aircraft is already airborne and idle enough to spare it. This is
        a BIAS for the caller to apply to its own attitude target, not a command in itself - the
        scanner deliberately has no authority to move the aircraft.
        """
        if busy or not airborne or dome is None:
            return 0.0
        if dome.coverage() >= self.COVERAGE_TARGET:
            return 0.0
        now = now or time.time()
        # A slow triangle wave: gentle enough not to disturb the flight path, wide enough that the
        # ring sweeps several elevation bins.
        phase = (now % self.TILT_PERIOD_S) / self.TILT_PERIOD_S
        tri = 4.0 * abs(phase - 0.5) - 1.0            # -1 .. +1
        return float(self.TILT_BIAS_DEG * tri)

    # ---- reporting ---------------------------------------------------------
    def status(self, dome):
        cov = (dome.coverage() * 100.0) if dome is not None else 0.0
        if self._active:
            return "SCAN: sweeping (%.0f%% of the dome known)" % cov
        return "SCAN: idle (%.0f%% known%s)" % (
            cov, "" if not self._suspended_reason else ", " + self._suspended_reason)
