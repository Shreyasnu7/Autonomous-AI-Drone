# laptop_ai/spatial_grid.py
"""
2.5D Spatial Awareness Grid — Fuses all sensors into a simple obstacle map.

Combines:
- YDLidar X2 (360 deg 2D scan, up to 8m)
- 4x VL53L1X ToF (front/right/back/left, up to 4m)
- MiDaS monocular depth (forward-facing, relative depth)
- Drone altitude (barometer/GPS)

Produces a human-readable spatial description for the LLM,
and a numeric grid for Pi0/ER Brain path planning.

Grid is 10x10 centered on the drone, each cell = 1m x 1m.
Values: 0 = free, 1 = obstacle detected.
Height info from altitude sensor only (2.5D, not full 3D).
"""

import math
import time
import cv2
import numpy as np
import logging

logger = logging.getLogger('SpatialGrid')


class SpatialGrid:
    GRID_SIZE = 50       # 50x50 cells = 10m x 10m at 20cm resolution (cm-accurate)
    CELL_SIZE = 0.2      # 0.2 m (20 cm) per cell
    CENTER = 25          # Drone is at (25, 25) in the grid

    # Render constants
    MAP_PX = 500         # 500x500 pixel output image
    CELL_PX = 50         # 500 / 10 = 50px per cell

    def __init__(self):
        self.grid = [[0] * self.GRID_SIZE for _ in range(self.GRID_SIZE)]
        self.altitude = 0.0
        self.last_update = 0
        self._obstacle_summary = {}
        # Raw sensor storage for visualization
        self._raw_lidar = []       # [(x, y), ...] in meters
        self._raw_tof = {}         # {t1: mm, t2: mm, ...}
        self._raw_depth = {}       # {subject_depth_m: float}
        self._raw_camera = []      # [(x, y), ...] camera-depth obstacle points (meters)
        # PRECISION CLOUD: keep the last few raw scans and render them ALL (older = dimmer).
        # Scan-to-scan sensor noise gives the natural dense grainy look of a real LiDAR
        # viewer — the X2 is a precise sensor and the feed should show it, not 20cm blocks.
        self._scan_history = []    # list of scans (each = list of (x, y)), newest LAST
        self.SCAN_TRAIL = 6
        # SHORT-TERM OBSTACLE MEMORY: the camera's depth cone only covers ~90 deg — once the
        # drone/gimbal turns away, those obstacles used to VANISH from the grid instantly.
        # Remember them for MEM_SECS, ego-compensated by the drone's own motion (forward speed +
        # yaw delta), and drop them the moment the camera re-observes their sector (fresh wins).
        self.MEM_SECS = 3.0
        self._mem = []             # [(x, y, t_seen)] camera-depth points, body frame
        self._prev_upd_t = None
        self._prev_yaw = None
        self._drone_yaw = 0.0     # radians
        self._heading_deg = 0.0
        self._speed = 0.0
        self._battery = 0

    def clear(self):
        for r in range(self.GRID_SIZE):
            for c in range(self.GRID_SIZE):
                self.grid[r][c] = 0
        self._obstacle_summary = {}

    def update(self, lidar_points=None, tof_sensors=None, depth_info=None,
               camera_points=None, altitude=0, drone_yaw_rad=0):
        """
        Fuse all sensor data into the grid.

        Args:
            lidar_points: list of (x, y) in meters, drone-centered (LiDAR: above, horizontal)
            tof_sensors: dict {t1: mm_front, t2: mm_right, t3: mm_back, t4: mm_left}
            depth_info: dict {subject_depth_m: float, depth_map_summary: str}
            camera_points: list of (x, y) in meters, drone-centered, from monocular depth
                           (camera: below, looking forward — fills the lidar's lower blind spot)
            altitude: meters above ground
            drone_yaw_rad: heading in radians (0 = north)
        """
        self.clear()
        self.altitude = altitude
        now = time.time()
        # --- SHORT-TERM MEMORY maintenance (ego-compensate, expire, re-observe) ---
        dt = min(0.5, max(0.0, now - self._prev_upd_t)) if self._prev_upd_t else 0.0
        dyaw = (drone_yaw_rad - self._prev_yaw) if self._prev_yaw is not None else 0.0
        if self._mem:
            # rotation compensation is dt-independent (a yaw is a yaw); only translation needs dt
            fwd = self._speed * dt                  # forward motion -> obstacles slide toward +y (back)
            c, s = math.cos(-dyaw), math.sin(-dyaw)  # drone yawed by dyaw -> rotate points opposite
            self._mem = [(x*c - y*s, x*s + y*c + fwd, t) for x, y, t in self._mem
                         if now - t <= self.MEM_SECS]
        if camera_points:
            # fresh camera data replaces memory inside the freshly-observed wedge
            bearings = [math.atan2(x, -y) for x, y in camera_points if (x or y)]
            if bearings:
                lo, hi = min(bearings) - 0.12, max(bearings) + 0.12
                self._mem = [(x, y, t) for x, y, t in self._mem
                             if not (lo <= math.atan2(x, -y) <= hi)]
            self._mem.extend((float(x), float(y), now) for x, y in camera_points)
            if len(self._mem) > 1200:
                self._mem = self._mem[-1200:]
        self._prev_upd_t = now
        self._prev_yaw = drone_yaw_rad
        self.last_update = now
        self._raw_lidar = lidar_points or []
        self._raw_tof = tof_sensors or {}
        self._raw_depth = depth_info or {}
        self._raw_camera = camera_points or []
        self._drone_yaw = drone_yaw_rad
        if lidar_points:
            self._scan_history.append(list(lidar_points))
            if len(self._scan_history) > self.SCAN_TRAIL:
                self._scan_history.pop(0)

        # 1. LiDAR points (already in drone body frame x,y meters)
        if lidar_points:
            for x, y in lidar_points:
                gx = int(self.CENTER + x / self.CELL_SIZE)
                gy = int(self.CENTER + y / self.CELL_SIZE)
                if 0 <= gx < self.GRID_SIZE and 0 <= gy < self.GRID_SIZE:
                    self.grid[gy][gx] = 1

        # 1b. CAMERA depth points (already projected into body frame, forward = -y).
        # Overlaid onto the SAME grid as the lidar -> the two sensors register together,
        # giving full vertical coverage (lidar's horizontal slice + camera's lower cone).
        if camera_points or self._mem:
            for x, y in list(camera_points or []) + [(mx, my) for mx, my, _t in self._mem]:
                gx = int(self.CENTER + x / self.CELL_SIZE)
                gy = int(self.CENTER + y / self.CELL_SIZE)
                if 0 <= gx < self.GRID_SIZE and 0 <= gy < self.GRID_SIZE:
                    self.grid[gy][gx] = 1

        # 2. ToF sensors (4 directional distances in mm)
        if tof_sensors:
            tof_dirs = {
                't1': (0, -1),    # front = negative y (forward in grid)
                't2': (1, 0),     # right = positive x
                't3': (0, 1),     # back = positive y
                't4': (-1, 0),    # left = negative x
            }
            for key, (dx, dy) in tof_dirs.items():
                dist_mm = tof_sensors.get(key, -1)
                if 0 < dist_mm < 4000:  # Valid ToF range
                    dist_m = dist_mm / 1000.0
                    # Mark cells along the ray up to the obstacle
                    gx = int(self.CENTER + dx * dist_m / self.CELL_SIZE)
                    gy = int(self.CENTER + dy * dist_m / self.CELL_SIZE)
                    if 0 <= gx < self.GRID_SIZE and 0 <= gy < self.GRID_SIZE:
                        self.grid[gy][gx] = 1

        # 3. MiDaS depth (forward-facing only — mark obstacles in front)
        if depth_info and depth_info.get('subject_depth_m', 99) < 8:
            depth_m = depth_info['subject_depth_m']
            # Obstacle in the forward direction at depth_m meters
            gy = int(self.CENTER - depth_m / self.CELL_SIZE)  # forward = -y
            gx = self.CENTER  # centered
            if 0 <= gy < self.GRID_SIZE:
                self.grid[gy][gx] = 1

        # Build summary
        self._build_summary()

    def _build_summary(self):
        """Per-direction nearest obstacle in CENTIMETERS, from RAW sensors (full precision).

        Computed directly from raw LiDAR points + ToF (mm) — NOT the quantized grid —
        so distances keep cm/mm accuracy instead of snapping to a cell.
        """
        directions = {
            'front': [], 'front_left': [], 'front_right': [],
            'left': [], 'right': [],
            'back': [], 'back_left': [], 'back_right': [],
        }

        # Raw obstacle samples (meters, drone-centered) — full sensor precision.
        # Camera points: FRESH cone + the SHORT-TERM MEMORY (obstacles the camera saw within
        # MEM_SECS that the drone/gimbal has since turned away from — no more amnesia).
        samples = list(self._raw_lidar)                    # LiDAR (cm-accurate x,y)
        samples.extend(self._raw_camera)
        samples.extend((x, y) for x, y, _t in self._mem)
        tof_dirs = {'t1': (0, -1), 't2': (1, 0), 't3': (0, 1), 't4': (-1, 0)}
        for key, (dx, dy) in tof_dirs.items():
            mm = self._raw_tof.get(key, -1)
            if 0 < mm < 4000:                              # VL53L1X: mm precision
                samples.append((dx * mm / 1000.0, dy * mm / 1000.0))
        dm = self._raw_depth.get('subject_depth_m', 99)
        if dm < 8:
            samples.append((0.0, -dm))                     # depth = forward

        for rx, ry in samples:
            dist = math.hypot(rx, ry)
            if dist < 0.15:
                continue  # within 15 cm = drone body

            angle = math.degrees(math.atan2(rx, -ry))  # 0=front, 90=right
            if -22.5 <= angle < 22.5:
                d = 'front'
            elif 22.5 <= angle < 67.5:
                d = 'front_right'
            elif 67.5 <= angle < 112.5:
                d = 'right'
            elif 112.5 <= angle < 157.5:
                d = 'back_right'
            elif angle >= 157.5 or angle < -157.5:
                d = 'back'
            elif -157.5 <= angle < -112.5:
                d = 'back_left'
            elif -112.5 <= angle < -67.5:
                d = 'left'
            else:
                d = 'front_left'

            directions[d].append(dist * 100.0)             # store in CM

        # Store FULL float precision (cm) — no rounding, nothing discarded.
        # Display rounds to the sensor's real limit (~1 mm = 0.1 cm); raw stays exact.
        self._obstacle_summary = {}
        for d, dists in directions.items():
            if dists:
                self._obstacle_summary[d] = min(dists)

    def get_spatial_description(self):
        """
        Returns a concise text description for the LLM.
        Example: "OBSTACLES: front=230cm, right=110cm | CLEAR: left, back | ALT=5.2m"
        """
        obstacles = []
        clear = []

        for d in ['front', 'front_left', 'front_right', 'left', 'right',
                   'back', 'back_left', 'back_right']:
            if d in self._obstacle_summary:
                obstacles.append(f"{d}={self._obstacle_summary[d]:.1f}cm")
            else:
                clear.append(d)

        parts = []
        if obstacles:
            parts.append(f"OBSTACLES: {', '.join(obstacles)}")
        if clear:
            parts.append(f"CLEAR: {', '.join(clear)}")
        parts.append(f"ALT={self.altitude:.1f}m")

        return " | ".join(parts)

    def get_closest_obstacle(self):
        """Returns (distance_cm, direction) of closest obstacle, or (9999, 'none')."""
        if not self._obstacle_summary:
            return 9999.0, 'none'
        closest_dir = min(self._obstacle_summary, key=self._obstacle_summary.get)
        return self._obstacle_summary[closest_dir], closest_dir

    def get_front_obstacle_m(self):
        """Nearest obstacle in the forward cone (front + front-left + front-right), in METERS.
        Lidar/ToF-backed (accurate). Returns 99.0 if the forward cone is clear."""
        fronts = [self._obstacle_summary[d] for d in ('front', 'front_left', 'front_right')
                  if d in self._obstacle_summary]
        return (min(fronts) / 100.0) if fronts else 99.0

    def get_grid_flat(self):
        """Returns flat list of 100 ints (0/1) for numeric consumers like Pi0."""
        return [cell for row in self.grid for cell in row]

    # The summary is keyed front/back/left/right, but the pilot vocabulary elsewhere in the
    # system is fwd/back/left/right (_HEAD_AZ). Map both so a caller cannot miss by a synonym.
    _DIR_ALIASES = {
        "fwd": "front", "forward": "front", "ahead": "front", "f": "front", "front": "front",
        "back": "back", "backward": "back", "behind": "back", "rear": "back", "b": "back",
        "left": "left", "port": "left", "l": "left",
        "right": "right", "starboard": "right", "r": "right",
    }

    def is_direction_clear(self, direction, min_distance_cm=200.0):
        """True if `direction` is clear of obstacles beyond min_distance_cm.

        Fails CLOSED. This previously defaulted an unrecognised key to 9999 cm, so asking about
        'fwd' -- the word the pilot model and resolver actually use -- reported CLEAR while the
        grid held an obstacle at 120 cm in front. A safety predicate must not answer "clear"
        for a question it did not understand.
        """
        key = self._DIR_ALIASES.get(str(direction or "").lower().strip())
        if key is None:
            logger.warning(f"is_direction_clear: unknown direction {direction!r} -> reporting BLOCKED")
            return False
        if key not in self._obstacle_summary:
            return True          # known direction, nothing recorded there = clear
        return self._obstacle_summary[key] >= min_distance_cm

    def set_telemetry(self, heading_deg=0, speed=0, battery=0):
        """Update telemetry for HUD overlay."""
        self._heading_deg = heading_deg
        self._speed = speed
        self._battery = battery

    def _m_to_px(self, mx, my):
        """Convert meters (drone-centered) to pixel coords on the flat map (legacy)."""
        px = int(self.CENTER * self.CELL_PX + mx * self.CELL_PX)
        py = int(self.CENTER * self.CELL_PX + my * self.CELL_PX)
        return px, py

    # ---------------- 2.5D ISOMETRIC RENDER ----------------
    # Canvas + isometric projection constants
    ISO_W = 640          # map area width
    ISO_H = 560          # canvas height
    SIDEBAR = 110        # altitude bar area
    TW = 27              # iso tile half-width  (x screen scale)
    TH = 14              # iso tile half-height (y screen scale, 2:1 iso)
    ZH = 30              # vertical pixels per meter of height (extrusion)
    OX = 320             # screen origin x (drone center)
    OY = 175             # screen origin y (drone center, ground plane)

    def _iso(self, mx, my, h=0.0):
        """Project drone-centered meters (mx=right, my=back) + height h to screen px."""
        sx = self.OX + (mx - my) * self.TW
        sy = self.OY + (mx + my) * self.TH - h * self.ZH
        return int(round(sx)), int(round(sy))

    @staticmethod
    def _dist_color(d, dmax=8.0):
        """Turbo-colormap BGR tuple for a distance (near=warm, far=cool) — point-cloud look."""
        t = max(0.0, min(1.0, d / dmax))
        v = np.uint8(255 * (1.0 - t))            # near -> high -> red/orange end of turbo
        c = cv2.applyColorMap(np.array([[v]], dtype=np.uint8), cv2.COLORMAP_TURBO)[0, 0]
        return int(c[0]), int(c[1]), int(c[2])

    @staticmethod
    def _shade(color, f):
        return tuple(int(c * f) for c in color)

    def _draw_iso_bar(self, img, mx, my, h, color):
        """Draw an extruded 3D obstacle bar (cube) at meters (mx,my) of height h meters."""
        s = 0.45 * self.CELL_SIZE
        # +y (front) face
        fy = np.array([self._iso(mx - s, my + s, 0), self._iso(mx + s, my + s, 0),
                       self._iso(mx + s, my + s, h), self._iso(mx - s, my + s, h)], np.int32)
        # +x (right) face
        fx = np.array([self._iso(mx + s, my - s, 0), self._iso(mx + s, my + s, 0),
                       self._iso(mx + s, my + s, h), self._iso(mx + s, my - s, h)], np.int32)
        top = np.array([self._iso(mx - s, my - s, h), self._iso(mx + s, my - s, h),
                        self._iso(mx + s, my + s, h), self._iso(mx - s, my + s, h)], np.int32)
        cv2.fillConvexPoly(img, fy, self._shade(color, 0.55))
        cv2.fillConvexPoly(img, fx, self._shade(color, 0.40))
        cv2.fillConvexPoly(img, top, color)
        cv2.polylines(img, [top], True, self._shade(color, 1.0), 1, cv2.LINE_AA)

    def render_map(self):
        """
        Render a 2.5D isometric spatial-awareness map (BGR numpy image).

        - Perspective ground grid (10x10 m)
        - LiDAR plotted as a distance-colored point cloud (turbo)
        - Obstacles extruded as 3D bars
        - ToF rays + endpoints, MiDaS depth marker
        - Drone body with heading arrow
        - Altitude bar, telemetry HUD, legend, freshness dot
        """
        S = self.ISO_W
        H = self.ISO_H
        img = np.zeros((H, S + self.SIDEBAR, 3), dtype=np.uint8)
        img[:] = (24, 22, 20)
        img[:, S:] = (32, 30, 28)
        cv2.rectangle(img, (0, 0), (S, 40), (18, 16, 14), -1)

        R = (self.GRID_SIZE * self.CELL_SIZE) / 2.0  # half-extent in meters (5.0)

        # --- GROUND GRID (isometric, 1 m steps) ---
        ri = int(R)
        for i in range(-ri, ri + 1):
            cv2.line(img, self._iso(i, -R, 0), self._iso(i, R, 0), (55, 52, 48), 1, cv2.LINE_AA)
            cv2.line(img, self._iso(-R, i, 0), self._iso(R, i, 0), (55, 52, 48), 1, cv2.LINE_AA)
        border = np.array([self._iso(-R, -R), self._iso(R, -R),
                           self._iso(R, R), self._iso(-R, R)], np.int32)
        cv2.polylines(img, [border], True, (90, 85, 80), 1, cv2.LINE_AA)

        # --- PRECISION LIDAR CLOUD (dense, grainy, cm-accurate) ---
        # Every raw return from the last SCAN_TRAIL scans is drawn as a fine point at its EXACT
        # position — no 20cm quantization. Older scans fade; the natural scan-to-scan noise of
        # the X2 builds the dense grainy surface a real LiDAR viewer shows.
        n_scans = max(1, len(self._scan_history))
        for si, scan in enumerate(self._scan_history or [self._raw_lidar]):
            age = (si + 1) / n_scans                      # oldest ~1/n ... newest = 1.0
            for x, y in scan:
                d = math.hypot(x, y)
                col = self._shade(self._dist_color(d), 0.25 + 0.75 * age)
                ppx, ppy = self._iso(x, y, 0.05)
                if age == 1.0:                            # newest scan: point + tiny lift tick
                    tpx, tpy = self._iso(x, y, min(0.10 + d * 0.02, 0.28))
                    cv2.line(img, (ppx, ppy), (tpx, tpy), self._shade(col, 0.45), 1, cv2.LINE_AA)
                    img[max(0, tpy), max(0, min(img.shape[1] - 1, tpx))] = col
                    cv2.circle(img, (tpx, tpy), 1, col, -1, cv2.LINE_AA)
                else:                                     # history: 1px grain on the ground plane
                    img[max(0, ppy), max(0, min(img.shape[1] - 1, ppx))] = col

        # --- DEPTH-AI CAMERA CLOUD (dense magenta sheet — per-pixel metric depth, cm-accurate) ---
        # The depth model sees a DENSE 90-deg forward cone (per-pixel). Drawn ABOVE the lidar trace
        # at a slight lift with its FOV wedge, so the camera's dense volume is clearly visible and
        # distinguishable from the sparse 360 ring.
        if self._raw_camera:
            yawr = self._drone_yaw
            for edge in (-math.pi / 4, math.pi / 4):               # faint 90-deg FOV wedge
                ex = math.sin(yawr + edge) * 4.0
                ey = -math.cos(yawr + edge) * 4.0
                cv2.line(img, self._iso(0, 0, 0.15), self._iso(ex, ey, 0.15), (90, 0, 90), 1, cv2.LINE_AA)
        for x, y in self._raw_camera:
            d = math.hypot(x, y)
            shade = max(0.45, 1.0 - d / 9.0)
            col = (int(255 * shade), 40, int(255 * shade))         # magenta = Depth (legend)
            ppx, ppy = self._iso(x, y, 0.15)                       # lifted above the lidar plane
            cv2.circle(img, (ppx, ppy), 1, col, -1, cv2.LINE_AA)
            gpx, gpy = self._iso(x, y, 0.03)
            img[max(0, gpy), max(0, min(img.shape[1] - 1, gpx))] = (int(110 * shade), 0, int(110 * shade))
        # remembered (out-of-view) depth obstacles: dim violet grain fading with age
        now2 = time.time()
        for x, y, t0 in self._mem:
            age = max(0.0, min(1.0, (now2 - t0) / max(0.1, self.MEM_SECS)))
            g = int(150 * (1.0 - 0.7 * age))
            ppx, ppy = self._iso(x, y, 0.03)
            img[max(0, ppy), max(0, min(img.shape[1] - 1, ppx))] = (g, 0, g)

        # --- FUSED CELLS: faint wireframe footprints only (the decision grid, not the visual) ---
        for gy in range(self.GRID_SIZE):
            for gx in range(self.GRID_SIZE):
                if self.grid[gy][gx] != 1:
                    continue
                mx = (gx - self.CENTER) * self.CELL_SIZE
                my = (gy - self.CENTER) * self.CELL_SIZE
                if abs(mx) < 0.2 and abs(my) < 0.2:
                    continue
                h = self.CELL_SIZE / 2
                foot = np.array([self._iso(mx - h, my - h), self._iso(mx + h, my - h),
                                 self._iso(mx + h, my + h), self._iso(mx - h, my + h)], np.int32)
                cv2.polylines(img, [foot], True,
                              self._shade(self._dist_color(math.hypot(mx, my)), 0.35), 1, cv2.LINE_AA)

        # --- TOF RAYS (4 directional) ---
        tof_dirs = {'t1': (0, -1), 't2': (1, 0), 't3': (0, 1), 't4': (-1, 0)}
        tof_labels = {'t1': 'F', 't2': 'R', 't3': 'B', 't4': 'L'}
        d0 = self._iso(0, 0, 0.2)
        for key, (dx, dy) in tof_dirs.items():
            dist_mm = self._raw_tof.get(key, -1)
            if 0 < dist_mm < 4000:
                dm = dist_mm / 1000.0
                end = self._iso(dx * dm, dy * dm, 0.2)
                cv2.line(img, d0, end, (0, 210, 210), 1, cv2.LINE_AA)
                cv2.circle(img, end, 4, (0, 0, 255), -1, cv2.LINE_AA)
                cv2.putText(img, f"{tof_labels[key]}:{dist_mm:.0f}mm", (end[0] + 6, end[1] - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.34, (0, 220, 220), 1, cv2.LINE_AA)

        # --- MIDAS DEPTH MARKER (forward, magenta) ---
        depth_m = self._raw_depth.get('subject_depth_m', 99)
        if depth_m < 8:
            mp = self._iso(0, -depth_m, 0.3)
            cv2.drawMarker(img, mp, (255, 0, 255), cv2.MARKER_TRIANGLE_UP, 12, 2, cv2.LINE_AA)
            cv2.putText(img, f"D:{depth_m*100:.1f}cm", (mp[0] + 8, mp[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.34, (255, 0, 255), 1, cv2.LINE_AA)

        # --- DRONE BODY + HEADING ARROW ---
        base = self._iso(0, 0, 0)
        body = self._iso(0, 0, 0.45)
        cv2.line(img, base, body, (120, 120, 120), 1, cv2.LINE_AA)
        cv2.ellipse(img, base, (self.TW, self.TH), 0, 0, 360, (70, 70, 70), 1, cv2.LINE_AA)
        cv2.circle(img, body, 6, (255, 255, 255), -1, cv2.LINE_AA)
        cv2.circle(img, body, 6, (0, 255, 0), 1, cv2.LINE_AA)
        yaw = self._drone_yaw
        tip = self._iso(math.sin(yaw) * 1.6, -math.cos(yaw) * 1.6, 0.45)  # forward = -y
        cv2.arrowedLine(img, body, tip, (0, 255, 0), 2, cv2.LINE_AA, tipLength=0.3)

        # --- ALTITUDE BAR ---
        bx = S + 25; bt = 50; bb = H - 50; bh = bb - bt
        cv2.rectangle(img, (bx, bt), (bx + 28, bb), (60, 58, 55), -1)
        cv2.rectangle(img, (bx, bt), (bx + 28, bb), (100, 98, 95), 1)
        max_alt = 50.0
        ay = int(bb - min(self.altitude / max_alt, 1.0) * bh)
        cv2.rectangle(img, (bx, ay), (bx + 28, bb), (0, 180, 0), -1)
        cv2.line(img, (bx - 4, ay), (bx + 32, ay), (0, 255, 0), 2)
        cv2.putText(img, f"{self.altitude:.1f}m", (bx - 8, ay - 7),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(img, "ALT", (bx + 2, bt - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
        for tick in [10, 20, 30, 40, 50]:
            ty = int(bb - (tick / max_alt) * bh)
            cv2.line(img, (bx + 28, ty), (bx + 34, ty), (100, 98, 95), 1)
            cv2.putText(img, f"{tick}", (bx + 36, ty + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (110, 108, 105), 1)

        # --- HUD ---
        hy = H - 22
        cv2.putText(img, f"HDG:{self._heading_deg:.0f}", (12, hy), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (210, 210, 210), 1, cv2.LINE_AA)
        cv2.putText(img, f"SPD:{self._speed:.1f}m/s", (140, hy), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (210, 210, 210), 1, cv2.LINE_AA)
        bc = (0, 255, 0) if self._battery > 30 else (0, 165, 255) if self._battery > 15 else (0, 0, 255)
        cv2.putText(img, f"BAT:{self._battery}%", (300, hy), cv2.FONT_HERSHEY_SIMPLEX, 0.45, bc, 1, cv2.LINE_AA)
        closest_d, closest_dir = self.get_closest_obstacle()
        if closest_d < 9999:
            cc = (0, 0, 255) if closest_d < 150 else (0, 165, 255) if closest_d < 300 else (0, 255, 0)
            cv2.putText(img, f"NEAREST {closest_dir} {closest_d:.1f}cm", (12, hy - 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, cc, 1, cv2.LINE_AA)

        # --- TITLE + LEGEND + FRESHNESS ---
        cv2.putText(img, "2.5D SPATIAL FUSION (cm)", (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (235, 235, 235), 1, cv2.LINE_AA)
        lx = S - 150
        cv2.circle(img, (lx, 16), 4, self._dist_color(1.0), -1); cv2.putText(img, "LiDAR near->far", (lx + 10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.circle(img, (lx, 32), 4, (0, 210, 210), -1); cv2.putText(img, "ToF", (lx + 10, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.circle(img, (lx, 48), 4, (255, 0, 255), -1); cv2.putText(img, "Depth", (lx + 10, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (200, 200, 200), 1, cv2.LINE_AA)
        age = time.time() - self.last_update if self.last_update else 999
        fc = (0, 255, 0) if age < 1 else (0, 165, 255) if age < 5 else (0, 0, 255)
        cv2.circle(img, (S + self.SIDEBAR - 16, 16), 6, fc, -1)

        return img


if __name__ == "__main__":
    # Standalone demo: synthesize sensor data and save a preview PNG so the look
    # can be verified without the live drone/AI running.
    import os
    sg = SpatialGrid()

    lidar = []
    for ang in range(0, 360, 3):
        a = math.radians(ang)
        r = 4.2 + 0.6 * math.sin(a * 3)
        lidar.append((r * math.cos(a), r * math.sin(a)))
    for _ in range(60):
        lidar.append((1.8 + np.random.uniform(-0.3, 0.3), -2.2 + np.random.uniform(-0.3, 0.3)))
    for _ in range(40):
        lidar.append((-2.6 + np.random.uniform(-0.25, 0.25), 0.4 + np.random.uniform(-0.5, 0.5)))

    sg.update(lidar_points=lidar,
              tof_sensors={'t1': 2100, 't2': 1300, 't3': 3500, 't4': 2600},
              depth_info={'subject_depth_m': 2.4},
              altitude=12.5, drone_yaw_rad=math.radians(35))
    sg.set_telemetry(heading_deg=35, speed=3.2, battery=68)

    img = sg.render_map()
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "spatial_map_preview.png")
    cv2.imwrite(out, img)
    print("Saved preview:", out, "| size:", img.shape)
    print("Spatial:", sg.get_spatial_description())
