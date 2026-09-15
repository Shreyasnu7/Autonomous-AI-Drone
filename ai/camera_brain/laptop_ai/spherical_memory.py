"""Spherical obstacle memory - the dome the flat grid cannot represent.

The spatial grid is a horizontal slice: it answers "what is two metres ahead" but not "is that
thing above or below my flight path". The LiDAR is a single horizontal plane and the camera sees
only the cone it is pointed at, so neither alone knows the shape of the space around the aircraft.

This accumulates BOTH into one body-frame dome binned by azimuth and elevation, so pointing the
gimbal somewhere, or tilting the airframe, adds lasting knowledge of that direction instead of it
being forgotten the moment the sensor looks away.

The lessons from the 2026-09-15 audit are built in deliberately:

  * entries carry a timestamp and EXPIRE. An obstacle seen once is not an obstacle forever; a map
    that never forgets fills with phantoms until nothing can be flown.
  * the memory is ego-compensated for yaw exactly, every update. A body-frame memory that does not
    turn with the aircraft is wrong the instant it turns.
  * translation is compensated approximately, and that limitation is stated rather than hidden.
  * a reading that cannot be placed is dropped, never guessed.

Body frame, matching spatial_grid and the ToF mapping:
    azimuth   0 = forward, +90 = right, +-180 = behind
    elevation 0 = level,   positive = up, negative = down
"""
import math
import time


class SphericalMemory:
    AZ_BINS = 24                 # 15 degrees per bin around the circle
    EL_BINS = 9                  # 15 degrees per bin, -60..+60
    EL_MIN_DEG = -60.0
    EL_MAX_DEG = 60.0
    MEM_SECS = 6.0               # how long a sighting is trusted without re-observation
    MAX_RANGE_M = 12.0

    def __init__(self):
        self._cells = {}         # (az_idx, el_idx) -> [distance_m, stamp]

    # ---- binning -------------------------------------------------------------
    def _az_idx(self, az_deg):
        return int(math.floor(((az_deg + 180.0) % 360.0) / (360.0 / self.AZ_BINS))) % self.AZ_BINS

    def _el_idx(self, el_deg):
        if el_deg < self.EL_MIN_DEG or el_deg > self.EL_MAX_DEG:
            return None
        span = (self.EL_MAX_DEG - self.EL_MIN_DEG) / self.EL_BINS
        return min(self.EL_BINS - 1, int((el_deg - self.EL_MIN_DEG) / span))

    def _az_centre(self, idx):
        return -180.0 + (idx + 0.5) * (360.0 / self.AZ_BINS)

    def _el_centre(self, idx):
        span = (self.EL_MAX_DEG - self.EL_MIN_DEG) / self.EL_BINS
        return self.EL_MIN_DEG + (idx + 0.5) * span

    # ---- writing -------------------------------------------------------------
    def add(self, az_deg, el_deg, dist_m, now=None):
        """Record the nearest surface in one direction. Unusable input is ignored, not guessed."""
        try:
            d = float(dist_m)
        except (TypeError, ValueError):
            return False
        if d != d or d <= 0.05 or d > self.MAX_RANGE_M:
            return False
        ei = self._el_idx(float(el_deg))
        if ei is None:
            return False
        ai = self._az_idx(float(az_deg))
        now = now or time.time()
        cur = self._cells.get((ai, ei))
        # Keep the NEAREST reading: the closest surface is what matters for clearance, and a
        # farther return in the same direction usually means the beam slipped past an edge.
        if cur is None or d <= cur[0] or (now - cur[1]) > self.MEM_SECS * 0.5:
            self._cells[(ai, ei)] = [d, now]
        return True

    def add_camera_cone(self, points_body, gimbal_pitch_deg=0.0, now=None):
        """points_body: (x_right_m, y_m) in the GRID convention, where forward = -y.

        This takes the same convention as spatial_grid and add_lidar_ring deliberately. The
        director stores camera returns as (lat, -fwd); reading the second element as forward
        instead of negated-forward puts every camera obstacle directly BEHIND the aircraft,
        which is exactly the kind of quiet frame-convention error that only shows up in flight.

        A third element, if present, is height above the aircraft in metres and gives the true
        elevation. With two elements the elevation comes from where the camera was pointing,
        which is the entire reason sweeping the gimbal is worth doing.
        """
        n = 0
        for p in (points_body or []):
            try:
                if len(p) >= 3:
                    lat, y, up = float(p[0]), float(p[1]), float(p[2])
                    fwd = -y
                    rng = math.sqrt(lat * lat + fwd * fwd + up * up)
                    if rng <= 1e-6:
                        continue
                    el = math.degrees(math.asin(max(-1.0, min(1.0, up / rng))))
                else:
                    lat, y = float(p[0]), float(p[1])
                    fwd = -y
                    rng = math.hypot(lat, fwd)
                    el = float(gimbal_pitch_deg)
            except (TypeError, ValueError, IndexError):
                continue
            if self.add(math.degrees(math.atan2(lat, fwd)), el, rng, now=now):
                n += 1
        return n

    def add_lidar_ring(self, points_xy, roll_deg=0.0, pitch_deg=0.0, now=None):
        """A horizontal LiDAR plane, tilted with the airframe.

        The scanner only ever sees its own plane. As the aircraft pitches and rolls that plane
        sweeps through elevation, so recording each return at the elevation the attitude implies
        turns a 2D scanner into a coarse dome over time, with no extra hardware.
        points_xy uses the grid convention (x_right, y) with forward = -y.
        """
        n = 0
        pr, rr = math.radians(pitch_deg), math.radians(roll_deg)
        for p in (points_xy or []):
            try:
                x, y = float(p[0]), float(p[1])
            except (TypeError, ValueError, IndexError):
                continue
            fwd = -y
            rng = math.hypot(x, fwd)
            if rng <= 1e-6:
                continue
            az = math.degrees(math.atan2(x, fwd))
            ar = math.radians(az)
            # Tilt lifts or drops the ring: pitch acts on the forward component, roll on the side.
            el = math.degrees(pr * math.cos(ar) + rr * math.sin(ar))
            if self.add(az, el, rng, now=now):
                n += 1
        return n

    # ---- maintenance ---------------------------------------------------------
    def update_ego(self, dyaw_rad=0.0, fwd_m=0.0, right_m=0.0, now=None):
        """Rotate and approximately translate the dome with the aircraft, then expire old cells.

        Yaw is exact. Translation re-ranges each cell along its own bearing, which is right for
        something straight ahead or behind and increasingly wrong to the sides; over the few
        seconds a sighting lives that is far better than pretending the aircraft has not moved.
        Anything that ends up out of range is dropped rather than extrapolated.
        """
        now = now or time.time()
        out = {}
        for (ai, ei), (d, t) in self._cells.items():
            if now - t > self.MEM_SECS:
                continue
            ar = math.radians(self._az_centre(ai))
            if abs(fwd_m) > 1e-6 or abs(right_m) > 1e-6:
                d = d - (fwd_m * math.cos(ar) + right_m * math.sin(ar))
                if d <= 0.05 or d > self.MAX_RANGE_M:
                    continue
            ai2 = self._az_idx(self._az_centre(ai) - math.degrees(dyaw_rad))
            key = (ai2, ei)
            prev = out.get(key)
            if prev is None or d < prev[0]:
                out[key] = [d, t]
        self._cells = out

    # ---- reading -------------------------------------------------------------
    def clearance(self, az_deg, el_deg, half_width_deg=20.0):
        """Nearest known surface within a cone, or None if that direction is unobserved.

        None means UNKNOWN, not clear. Callers must treat the two differently.
        """
        best = None
        for (ai, ei), (d, _t) in self._cells.items():
            daz = abs(((self._az_centre(ai) - az_deg + 180.0) % 360.0) - 180.0)
            de = abs(self._el_centre(ei) - el_deg)
            if daz <= half_width_deg and de <= half_width_deg:
                if best is None or d < best:
                    best = d
        return best

    def coverage(self):
        """Fraction of the dome observed recently - how much of its surroundings it actually
        knows, rather than implying knowledge it does not have."""
        return len(self._cells) / float(self.AZ_BINS * self.EL_BINS)

    def unobserved_directions(self, el_deg=0.0, half_width_deg=20.0):
        """Azimuths at this elevation with no recent reading: where looking would pay."""
        return [round(self._az_centre(ai), 1) for ai in range(self.AZ_BINS)
                if self.clearance(self._az_centre(ai), el_deg, half_width_deg) is None]

    def describe(self, max_items=6):
        """Compact line for the pilot prompt: nearest surfaces and where they are."""
        items = sorted(((d, self._az_centre(ai), self._el_centre(ei))
                        for (ai, ei), (d, _t) in self._cells.items()), key=lambda r: r[0])
        if not items:
            return "DOME: nothing observed yet"
        parts = ["%.0fcm at az%+.0f el%+.0f" % (d * 100, az, el) for d, az, el in items[:max_items]]
        return "DOME (%.0f%% observed): %s" % (self.coverage() * 100, ", ".join(parts))
