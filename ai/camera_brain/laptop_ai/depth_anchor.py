# laptop_ai/depth_anchor.py
"""Motion-triangulation DEPTH SCALE ANCHOR — cm-class metric truth from the camera alone.

The monocular depth model gives a dense map whose SCALE can drift (2-10%). The drone is always
moving, so parallax is free: track sparse features between consecutive frames, remove the
ROTATIONAL flow (from the FC attitude deltas), and triangulate each feature's true depth from
the TRANSLATIONAL flow and the known baseline (the drone's own motion). The median ratio
(triangulated / model) is a robust SCALE correction for the whole dense map.

    flow_t = ( (u*t_z - f*t_x) / Z,  (v*t_z - f*t_y) / Z )      (pinhole, camera frame)
    =>  Z_i = (a . flow_t) / (flow_t . flow_t),  a = (u*t_z - f*t_x,  v*t_z - f*t_y)

Physics of the precision: sigma_Z ~ Z^2 * sigma_px / (f * B). GoPro (f~450px @512w), B=0.3m:
~1.5cm at 2m, ~9cm at 5m — cm-class, NOT mm (mm-from-video at range is marketing).

Fail-safe: returns None when there is not enough baseline (<2cm) or parallax (<1.5px median)
or too few tracked inliers — callers fall back to the ToF anchor / neutral 1.0."""
import math
import time
import cv2
import numpy as np


class DepthScaleAnchor:
    MIN_BASELINE_M = 0.02      # need >=2cm of translation between frames
    MIN_FLOW_PX = 1.5          # median translational flow must exceed this
    MIN_INLIERS = 8
    EMA = 0.25                 # smoothing on the recovered scale

    def __init__(self, hfov_deg=86.0, width_px=None):
        self.hfov = math.radians(hfov_deg)
        self._prev_gray = None
        self._prev_pts = None
        self.scale = None          # last fused scale (None until first lock)
        self.last_lock_t = 0.0
        self.stats = {"locks": 0, "rejects": 0}

    def _focal_px(self, w):
        return (w / 2.0) / math.tan(self.hfov / 2.0)

    def update(self, gray, model_depth_m, t_body, datt_rad):
        """gray: current frame (grayscale uint8). model_depth_m: dense metric map (H×W, metres,
        the MODEL's current belief incl. any previous scaling). t_body: (fwd, right, up) metres
        moved since the previous frame (body frame). datt_rad: (droll, dpitch, dyaw) attitude
        change since the previous frame. Returns the fused scale (float) or None if no lock."""
        try:
            h, w = gray.shape[:2]
            f = self._focal_px(w)
            if self._prev_gray is None:
                self._prev_gray = gray
                self._prev_pts = cv2.goodFeaturesToTrack(gray, 300, 0.01, 8)
                return self.scale
            prev_pts = self._prev_pts
            if prev_pts is None or len(prev_pts) < self.MIN_INLIERS:
                prev_pts = cv2.goodFeaturesToTrack(self._prev_gray, 300, 0.01, 8)
            if prev_pts is None or len(prev_pts) < self.MIN_INLIERS:
                self._roll(gray); self.stats["rejects"] += 1
                return self.scale

            # camera frame: x right, y down, z forward  (body fwd/right/up -> t_x=right, t_y=-up, t_z=fwd)
            t_x, t_y, t_z = float(t_body[1]), -float(t_body[2]), float(t_body[0])
            B = math.sqrt(t_x*t_x + t_y*t_y + t_z*t_z)
            if B < self.MIN_BASELINE_M:
                self._roll(gray); self.stats["rejects"] += 1
                return self.scale

            nxt, st, _ = cv2.calcOpticalFlowPyrLK(self._prev_gray, gray, prev_pts, None,
                                                  winSize=(21, 21), maxLevel=3)
            if nxt is None:
                self._roll(gray); self.stats["rejects"] += 1
                return self.scale
            ok = st.reshape(-1) == 1
            p0 = prev_pts.reshape(-1, 2)[ok]; p1 = nxt.reshape(-1, 2)[ok]
            if len(p0) < self.MIN_INLIERS:
                self._roll(gray); self.stats["rejects"] += 1
                return self.scale

            cx, cy = w / 2.0, h / 2.0
            u0, v0 = p0[:, 0] - cx, p0[:, 1] - cy
            flow = p1 - p0
            # ROTATION COMPENSATION (small-angle): yaw psi -> du ~ f*psi + u*v/f*...; keep the
            # dominant terms (exact enough for the <2deg/frame regime the loop runs at):
            droll, dpitch, dyaw = [float(a) for a in datt_rad]
            du_rot = f*dyaw + (u0*v0/f)*dpitch - v0*droll + (u0*u0/f)*dyaw*0.0
            dv_rot = -f*dpitch + (u0*v0/f)*dyaw*0.0 + u0*droll
            ft = flow - np.stack([du_rot, dv_rot], axis=1)
            mag = np.linalg.norm(ft, axis=1)
            if float(np.median(mag)) < self.MIN_FLOW_PX:
                self._roll(gray); self.stats["rejects"] += 1
                return self.scale

            # Z from translational flow (per-feature least squares along the predicted direction)
            a_u = u0*t_z - f*t_x
            a_v = v0*t_z - f*t_y
            denom = ft[:, 0]*ft[:, 0] + ft[:, 1]*ft[:, 1]
            num = a_u*ft[:, 0] + a_v*ft[:, 1]
            with np.errstate(divide="ignore", invalid="ignore"):
                Z = num / denom
            # model depth at the same pixels
            iy = np.clip(p0[:, 1].astype(int), 0, model_depth_m.shape[0]-1)
            ix = np.clip(p0[:, 0].astype(int), 0, model_depth_m.shape[1]-1)
            Zm = model_depth_m[iy, ix]
            good = (np.isfinite(Z) & (Z > 0.2) & (Z < 12.0) & np.isfinite(Zm) &
                    (Zm > 0.1) & (mag > self.MIN_FLOW_PX))
            if int(good.sum()) < self.MIN_INLIERS:
                self._roll(gray); self.stats["rejects"] += 1
                return self.scale
            ratios = (Z[good] / Zm[good])
            s = float(np.median(ratios))
            # robust: demand agreement (inlier spread) so junk flow can't move the scale
            spread = float(np.median(np.abs(ratios - s))) / max(s, 1e-6)
            if not (0.25 <= s <= 4.0) or spread > 0.35:
                self._roll(gray); self.stats["rejects"] += 1
                return self.scale
            self.scale = s if self.scale is None else (1-self.EMA)*self.scale + self.EMA*s
            self.last_lock_t = time.time()
            self.stats["locks"] += 1
            self._roll(gray)
            return self.scale
        except Exception:
            self._roll(gray)
            return self.scale

    def fresh(self, max_age_s=1.5):
        return self.scale is not None and (time.time() - self.last_lock_t) <= max_age_s

    def _roll(self, gray):
        self._prev_gray = gray
        try:
            self._prev_pts = cv2.goodFeaturesToTrack(gray, 300, 0.01, 8)
        except Exception:
            self._prev_pts = None
