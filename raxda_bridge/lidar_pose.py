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
    def __init__(self, res_m=0.10, size_m=24.0, map_hits_to_localize=60):
        self.res = res_m
        self.n = int(size_m / res_m)
        self.c = self.n // 2                       # world origin (takeoff point) at grid centre
        self.occ = np.zeros((self.n, self.n), np.uint8)   # 0 unknown, 2 occupied
        self.x = 0.0; self.y = 0.0; self.yaw = 0.0        # pose (m, m, deg) — origin = takeoff
        self._mapped = 0
        self._need = map_hits_to_localize

    # ---- grid helpers ----
    def _cell(self, x, y):
        return int(round(y / self.res)) + self.c, int(round(x / self.res)) + self.c

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
            if self.occ[r, cc] == 2:
                s += 2
            else:
                for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                    if self._inb(r + dr, cc + dc) and self.occ[r + dr, cc + dc] == 2:
                        s += 1
                        break
        return s

    def _localize(self, prior, heading, pts):
        best = (prior[0], prior[1])
        bs = self._score(best[0], best[1], heading, pts)
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
        return best

    def _integrate(self, pts, heading):
        hr = math.radians(heading)
        for a, d in pts:
            ar = math.radians(a)
            wx = self.x + d * math.sin(hr + ar)
            wy = self.y + d * math.cos(hr + ar)
            r, cc = self._cell(wx, wy)
            if self._inb(r, cc):
                self.occ[r, cc] = 2
        self._mapped = int((self.occ == 2).sum())

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
        return self.x, self.y, self.yaw

    def pose(self):
        return self.x, self.y, self.yaw
