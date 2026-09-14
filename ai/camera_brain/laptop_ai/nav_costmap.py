"""
ai/camera_brain/laptop_ai/nav_costmap.py — LOCAL (egocentric) costmap + A* path planner.

WHY THIS EXISTS (Part B, the user's deepest concern): the production pilot only had REACTIVE
avoidance (clamp/repulse the AI's velocity). If the VLM picked a wrong DIRECTION, nothing corrected
it. A planner makes the ROUTE code-owned: the VLM/goal says WHERE to head, this plans the safe PATH
there around obstacles it remembers — so a single bad VLM tick can't derail the flight.

WHY LOCAL/EGOCENTRIC (not the sim's global map): the real drone has NO GPS and no productionised
SLAM (get_position()->[0,0,0]). A global costmap needs a world position we don't have. A LOCAL
costmap is drone-centred: the drone is always at the grid centre, the map is SHIFTED by the FC's
short-horizon odometry (velocity*dt) each tick, and every cell DECAYS — so it needs no absolute
position, tolerates drift (only the last few metres matter), and naturally handles moving obstacles.

Inputs it fuses each tick: body-frame obstacle points (metres) from LiDAR + ToF + the metric-depth
camera (director_core builds these already). Output: (vx, vy, yaw_rate) toward a body-frame goal, or
None if boxed in (caller falls back to reactive avoidance).

Body frame convention (matches director_core / ToF grid): +x = RIGHT, +y = FORWARD.
This module is PURE algorithm (numpy) — unit-testable with no drone. Live behaviour still needs
on-hardware tuning (odometry quality, cell size, inflation, speed table).
"""
import math
import heapq
import numpy as np


class LocalCostmap:
    def __init__(self, size_m=6.0, res_m=0.15, drone_radius_m=0.25, decay=0.85, hit=1.0, occ_thresh=0.5):
        self.res = res_m
        self.n = int(round(size_m / res_m)) | 1          # odd so there is a true centre cell
        self.c0 = self.n // 2                            # drone is always at (c0, c0)
        self.occ = np.zeros((self.n, self.n), np.float32)  # obstacle confidence 0..~3
        self.decay = decay
        self.hit = hit
        self.occ_thresh = occ_thresh
        self.infl = max(1, int(math.ceil(drone_radius_m / res_m)))

    # ---- frame <-> cell ----
    def cell(self, x, y):
        """body-frame metres (x right, y fwd) -> (row, col). row DECREASES as forward increases so
        the array prints with forward = up."""
        c = self.c0 + int(round(x / self.res))
        r = self.c0 - int(round(y / self.res))
        return r, c

    def world(self, r, c):
        x = (c - self.c0) * self.res
        y = (self.c0 - r) * self.res
        return x, y

    def inb(self, r, c):
        return 0 <= r < self.n and 0 <= c < self.n

    # ---- per-tick update ----
    def rebuild(self, points):
        """PRIMARY update: clear the grid and mark the CURRENT fused obstacle points (metres, body
        frame). Fresh-each-tick = no odometry-warp smear and no ghost trails (the robust choice for a
        no-GPS drone whose sensors re-detect every tick). Persistent room memory needs SLAM — until
        then this plans on the live obstacle field, which still makes the ROUTE code-owned (Part B)."""
        self.occ.fill(0.0)
        self.mark(points)

    def shift(self, dx, dy, dyaw_rad):
        """OPTIONAL odometry-shift memory (drone stays centred). NOTE: warping a raster every tick
        accumulates smear — prefer rebuild() unless you specifically need multi-tick memory and have
        good odometry. Kept for experimentation."""
        """Move the map opposite to the drone's motion so the drone stays centred.
        dx,dy = body-frame displacement THIS tick (metres); dyaw = heading change (rad, +ccw)."""
        self.occ *= self.decay                            # fade stale/dynamic obstacles
        self.occ[self.occ < 0.05] = 0.0
        # rotation about centre by -dyaw (the world rotates opposite the drone's yaw)
        if abs(dyaw_rad) > 1e-3:
            deg = math.degrees(-dyaw_rad)
            try:
                import cv2
                M = cv2.getRotationMatrix2D((self.c0, self.c0), deg, 1.0)
                self.occ = cv2.warpAffine(self.occ, M, (self.n, self.n), flags=cv2.INTER_NEAREST,
                                          borderMode=cv2.BORDER_CONSTANT, borderValue=0.0)
            except Exception:
                pass
        # translation: drone moved (dx,dy) forward/right -> obstacles move (-dx,-dy) in the grid
        sc = int(round(-dx / self.res))                   # +x(right) -> shift cols left
        sr = int(round(dy / self.res))                    # +y(fwd)  -> obstacles move DOWN (row+)
        if sr or sc:
            self.occ = np.roll(self.occ, (sr, sc), axis=(0, 1))
            if sr > 0:   self.occ[:sr, :] = 0.0
            elif sr < 0: self.occ[sr:, :] = 0.0
            if sc > 0:   self.occ[:, :sc] = 0.0
            elif sc < 0: self.occ[:, sc:] = 0.0

    def mark(self, points):
        """points = iterable of (x_right, y_fwd) obstacle positions in metres (body frame)."""
        for x, y in points:
            r, c = self.cell(x, y)
            if self.inb(r, c):
                self.occ[r, c] = min(self.occ[r, c] + self.hit, 3.0)

    # ---- planning ----
    def _blocked(self, r, c):
        if not self.inb(r, c):
            return True
        r0, r1 = max(0, r - self.infl), min(self.n, r + self.infl + 1)
        c0, c1 = max(0, c - self.infl), min(self.n, c + self.infl + 1)
        return bool(np.any(self.occ[r0:r1, c0:c1] >= self.occ_thresh))

    def astar(self, goal_rc):
        """8-connected A* from the drone centre to goal_rc. Returns a list of cells or None."""
        start = (self.c0, self.c0)
        gr, gc = goal_rc
        if not self.inb(gr, gc):
            return None
        if self._blocked(gr, gc):
            goal_rc = self._nearest_free(gr, gc)          # nudge goal out of an obstacle
            if goal_rc is None:
                return None
            gr, gc = goal_rc
        openq = [(0.0, start)]
        came = {start: None}
        g = {start: 0.0}
        while openq:
            _, cur = heapq.heappop(openq)
            if cur == (gr, gc):
                break
            cr, cc = cur
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    if dr == 0 and dc == 0:
                        continue
                    nr, nc = cr + dr, cc + dc
                    if self._blocked(nr, nc):
                        continue
                    step = 1.41421 if (dr and dc) else 1.0
                    ng = g[cur] + step
                    if (nr, nc) not in g or ng < g[(nr, nc)]:
                        g[(nr, nc)] = ng
                        h = math.hypot(nr - gr, nc - gc)
                        heapq.heappush(openq, (ng + h, (nr, nc)))
                        came[(nr, nc)] = cur
        if (gr, gc) not in came:
            return None
        path = []
        n = (gr, gc)
        while n is not None:
            path.append(n)
            n = came[n]
        return path[::-1]

    def _nearest_free(self, r, c, max_r=8):
        for rad in range(1, max_r):
            for dr in range(-rad, rad + 1):
                for dc in range(-rad, rad + 1):
                    nr, nc = r + dr, c + dc
                    if self.inb(nr, nc) and not self._blocked(nr, nc):
                        return (nr, nc)
        return None

    # ---- controller ----
    def plan_velocity(self, goal_x, goal_y, max_speed=0.5, lookahead_m=0.7):
        """Plan a path to the body-frame goal (metres) and return (vx, vy, yaw_rate, path) via
        pure-pursuit. vx/vy in m/s (body frame), yaw_rate in deg/s. Returns (None,None,None,None)
        if no path exists (caller should fall back to reactive avoidance / stop)."""
        goal_rc = self.cell(goal_x, goal_y)
        path = self.astar(goal_rc)
        if not path or len(path) < 2:
            return None, None, None, None
        # pure-pursuit: first waypoint at least lookahead_m from the drone centre
        look = None
        for rc in path[1:]:
            wx, wy = self.world(*rc)
            if math.hypot(wx, wy) >= lookahead_m:
                look = (wx, wy)
                break
        if look is None:
            look = self.world(*path[-1])
        lx, ly = look
        bearing = math.atan2(lx, ly)                      # 0 = straight ahead, +right
        # slow down when the chosen heading is tight (nearest obstacle along +y within a cone)
        speed = max_speed * self._forward_clearance_factor()
        vx = speed * math.sin(bearing)
        vy = speed * math.cos(bearing)
        yaw_rate = max(-60.0, min(60.0, math.degrees(bearing) * 1.5))
        return vx, vy, yaw_rate, path

    def _forward_clearance_factor(self):
        """0..1 speed scale from how close the nearest obstacle is in the forward cone."""
        # nearest occupied cell in the +y (forward) half within ~±30°
        ys, xs = np.where(self.occ >= self.occ_thresh)
        best = 99.0
        for r, c in zip(ys, xs):
            x, y = self.world(r, c)
            if y > 0 and abs(math.degrees(math.atan2(x, y))) < 30:
                best = min(best, math.hypot(x, y))
        if best < 0.6:  return 0.0
        if best < 1.0:  return 0.4
        if best < 1.6:  return 0.7
        return 1.0

    # ---- debug ----
    def ascii(self):
        rows = []
        for r in range(self.n):
            line = ""
            for c in range(self.n):
                if (r, c) == (self.c0, self.c0): line += "D"
                elif self.occ[r, c] >= self.occ_thresh:  line += "#"
                elif self.occ[r, c] > 0.05:              line += "."
                else:                                    line += " "
            rows.append(line)
        return "\n".join(rows)
