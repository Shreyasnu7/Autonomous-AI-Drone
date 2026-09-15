"""
raxda_bridge/lidar_pose.py — 2D LiDAR SLAM for INDOOR (no-GPS) position.

Produces a live (x, y, yaw) pose from the YDLiDAR scans + the FC compass heading, so the bridge can feed
the flight controller a VISION_POSITION_ESTIMATE (ArduPilot ExternalNav / EK3_SRC = 6). That gives the
FC an indoor position → unlocks Loiter / PosHold / GUIDED position-hold + fixes 'PreArm: Need Position
Estimate' with NO GPS. Runs on the Radxa (next to the LiDAR + FC) for low latency.

Method (proven in sim — scratchpad/slam_sim.py, ~16cm drift): a drifty DEAD-RECKONING prior (integrate the
FC's velocity, heading from the compass — reliable) CORRECTED by SCAN-TO-MAP matching (Hector-SLAM style
correlative hill-climb against an accumulating occupancy map). Pure numpy; unit-testable without hardware.
NOTE: a simple correlative matcher — good for a demo/first flight; a production deploy should graduate to
Cartographer/hector_slam for loop-closure + robustness in featureless corridors.
"""
import math
import numpy as np


class LidarPoseEstimator:
    # Evidence thresholds. The map holds a CONFIDENCE COUNT per cell rather than a binary flag,
    # so an obstacle that stops being observed fades instead of becoming permanent.
    HIT_INC   = 3      # evidence added each time a beam ends in a cell
    HIT_MAX   = 6      # ceiling: a wall re-observed each scan stays pinned, but an
                       # obstacle that leaves clears in ~4 decay passes (~4 s at 10 Hz)
    OCC_THRESH = 3     # count at or above this = occupied
    DECAY_EVERY = 10   # scans between decay passes (~1 s at 10 Hz)
    EDGE_MARGIN_M = 3.0  # re-centre the map when the pose comes this close to its edge

    def __init__(self, res_m=0.10, size_m=24.0, map_hits_to_localize=60):
        self.res = res_m
        self.n = int(size_m / res_m)
        self.c = self.n // 2                       # grid centre
        # Confidence counts, not a binary occupancy flag (see HIT_INC/OCC_THRESH above).
        self.occ = np.zeros((self.n, self.n), np.int16)
        self.x = 0.0; self.y = 0.0; self.yaw = 0.0        # pose (m, m, deg) — origin = takeoff
        # The grid SCROLLS with the aircraft. Without this the map was a fixed +-12 m box and
        # everything outside it was silently dropped, so localisation quietly degraded on a
        # longer flight with no indication. These track where the grid centre sits in the
        # takeoff frame, so the returned pose stays in the original origin's coordinates.
        self.ox = 0.0; self.oy = 0.0
        self._mapped = 0
        self._need = map_hits_to_localize
        self._scans = 0

    # ---- grid helpers ----
    def _cell(self, x, y):
        return (int(round((y - self.oy) / self.res)) + self.c,
                int(round((x - self.ox) / self.res)) + self.c)

    def _inb(self, r, c):
        return 0 <= r < self.n and 0 <= c < self.n

    # ---- scan-to-map score (how well the scan, placed at (ex,ey,heading), hits known walls) ----
    def _score(self, ex, ey, heading, pts):
        s = 0
        hr = math.radians(heading)
        sinh, cosh = math.sin(hr), math.cos(hr)
        for a, d in pts:
            ar = math.radians(a)
            # beam endpoint in world frame (heading + beam angle)
            wx = ex + d * math.sin(hr + ar)
            wy = ey + d * math.cos(hr + ar)
            r, cc = self._cell(wx, wy)
            if not self._inb(r, cc):
                continue
            if self.occ[r, cc] >= self.OCC_THRESH:
                s += 2
            else:
                for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                    if self._inb(r + dr, cc + dc) and self.occ[r + dr, cc + dc] >= self.OCC_THRESH:
                        s += 1
                        break
        return s

    # A correction is only accepted when the scan clearly prefers it, and only if it is
    # physically reachable. Without these the matcher could drag the pose off a correct motion
    # prior on weak evidence -- in degenerate geometry (a featureless corridor, or a map built
    # while stationary) it scores highest at the position that CREATED the map and pins the pose
    # there. A frozen pose is the dangerous failure: the flight controller is told the aircraft
    # is stationary while it flies away, and position hold then pushes it further off.
    MATCH_MARGIN = 1.08        # scan-match must beat the prior by 8% to override it
    MAX_CORRECTION_M = 0.50    # a single scan may not teleport the pose further than this

    def _localize(self, prior, heading, pts):
        prior_score = self._score(prior[0], prior[1], heading, pts)
        best = (prior[0], prior[1])
        bs = prior_score
        for step in (0.10, 0.04):                  # coarse then fine hill-climb around the prior
            improved = True
            while improved:
                improved = False
                for i in (-1, 0, 1):
                    for j in (-1, 0, 1):
                        ex, ey = best[0] + i * step, best[1] + j * step
                        sc = self._score(ex, ey, heading, pts)
                        if sc > bs:
                            bs = sc; best = (ex, ey); improved = True
        if math.hypot(best[0] - prior[0], best[1] - prior[1]) > self.MAX_CORRECTION_M:
            return prior                            # implausible jump -> keep dead reckoning
        if bs < prior_score * self.MATCH_MARGIN:
            return prior                            # not clearly better -> keep dead reckoning
        return best

    def _integrate(self, pts, heading):
        hr = math.radians(heading)
        for a, d in pts:
            ar = math.radians(a)
            wx = self.x + d * math.sin(hr + ar)
            wy = self.y + d * math.cos(hr + ar)
            r, cc = self._cell(wx, wy)
            if self._inb(r, cc):
                self.occ[r, cc] = min(self.HIT_MAX, int(self.occ[r, cc]) + self.HIT_INC)
        self._mapped = int((self.occ >= self.OCC_THRESH).sum())

    def _decay(self):
        """Fade evidence that is no longer being re-observed.

        The map previously latched every hit forever, so anything transient -- a person walking
        through, a door that opened, the operator standing beside the aircraft -- became a
        permanent wall that the scan matcher then tried to localise against. One decrement per
        pass means a cell seen once clears in ~3 passes while a continuously re-observed wall
        stays pinned at HIT_MAX. Vectorised: one pass over the grid, not per-beam work.
        """
        np.subtract(self.occ, 1, out=self.occ, where=self.occ > 0)
        self._mapped = int((self.occ >= self.OCC_THRESH).sum())

    def _recenter(self):
        """Scroll the grid so the aircraft stays near the middle.

        The map is a fixed-size window; without this, flying beyond it meant every beam fell
        outside the array and was silently discarded, degrading localisation with no signal.
        Shifting by whole cells keeps the grid aligned so no resampling error accumulates, and
        the vacated band is cleared because it is genuinely unobserved. self.ox/oy keep the
        reported pose in the original takeoff frame.
        """
        dx_cells = int(round((self.x - self.ox) / self.res))
        dy_cells = int(round((self.y - self.oy) / self.res))
        if dx_cells == 0 and dy_cells == 0:
            return
        # occ is indexed [row=y, col=x]; a positive shift moves the world the other way.
        self.occ = np.roll(self.occ, (-dy_cells, -dx_cells), axis=(0, 1))
        if dy_cells > 0:
            self.occ[-min(dy_cells, self.n):, :] = 0
        elif dy_cells < 0:
            self.occ[:min(-dy_cells, self.n), :] = 0
        if dx_cells > 0:
            self.occ[:, -min(dx_cells, self.n):] = 0
        elif dx_cells < 0:
            self.occ[:, :min(-dx_cells, self.n)] = 0
        self.ox += dx_cells * self.res
        self.oy += dy_cells * self.res
        self._mapped = int((self.occ >= self.OCC_THRESH).sum())

    def update(self, scan, heading_deg, dt=0.1, odom_vel=(0.0, 0.0)):
        """scan = list of (angle_deg, dist_m) in the LiDAR frame; heading_deg = FC compass; odom_vel =
        (vx,vy) body m/s from the FC for the dead-reckon prior. Returns (x, y, yaw_deg)."""
        pts = [(a, d) for a, d in scan if 0.12 < d < 8.0]
        # dead-reckon prior: integrate body velocity into the world using the heading
        hr = math.radians(heading_deg)
        fu = (math.sin(hr), math.cos(hr))          # body-forward in world
        ru = (math.cos(hr), -math.sin(hr))         # body-right in world
        vx, vy = odom_vel
        prior = (self.x + (fu[0] * vx + ru[0] * vy) * dt,
                 self.y + (fu[1] * vx + ru[1] * vy) * dt)
        if self._mapped > self._need and pts:
            self.x, self.y = self._localize(prior, heading_deg, pts)
        else:
            self.x, self.y = prior
        self.yaw = heading_deg
        if pts:
            self._integrate(pts, heading_deg)
        self._scans += 1
        if self._scans % self.DECAY_EVERY == 0:
            self._decay()
        # Keep the aircraft inside the mapped window (see _recenter).
        half_m = (self.n * self.res) / 2.0
        if (abs(self.x - self.ox) > half_m - self.EDGE_MARGIN_M or
                abs(self.y - self.oy) > half_m - self.EDGE_MARGIN_M):
            self._recenter()
        return self.x, self.y, self.yaw

    def pose(self):
        return self.x, self.y, self.yaw
