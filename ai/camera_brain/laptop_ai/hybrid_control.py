"""
Hybrid autonomy control core (deterministic).

The AI (Gemini/Qwen) decides WHAT/WHERE (semantic intent); THIS module turns each intent into a
PRECISE, safe velocity setpoint using live sensor geometry + kinematics, and a deterministic
MissionSequencer tracks plan progress so the AI never has to self-sequence (which a small model
cannot do reliably). Pure python/math — no torch, no hardware, no network — so it's fast, testable,
and identical whether driven by the sim or the real drone.

Layers (each runs at its own rate; see project notes):
  Gemini  -> a PLAN = ordered list of Steps (each with a completion predicate)
  Sequencer (this) -> picks the CURRENT step, advances when its predicate is met
  Qwen    -> per step: confirms the semantic target / "done?" (optional; geometric steps need no AI)
  resolve_intent (this) -> intent + live sensors -> exact (vx,vy,vz,yaw_rate)
  local_avoid  (this) -> clamps that velocity so it can never close an obstacle inside stopping dist
  autopilot_controller.send_velocity -> ArduPilot -> motors (ArduPilot handles wind/attitude @400Hz)

INTENT schema (dict):
  {"maneuver": "HOLD|GOTO|ASCEND|DESCEND|SCAN|ORBIT|FOLLOW|WALL_FOLLOW",
   "bearing_deg": float,        # body-relative, 0=fwd +=right (GOTO/aim)
   "target_alt_m": float,       # ASCEND/DESCEND
   "target_bearing_deg": float, # ORBIT/FOLLOW: where the subject is (from perception)
   "target_dist_m": float,      # ORBIT radius / FOLLOW standoff
   "yaw_dir": "L|R|0",          # SCAN
   "side": "L|R",               # WALL_FOLLOW wall to keep
   "pace": "slow|norm",
   "conf": 0.0-1.0}

CTX schema (dict, live each tick):
  {"tof": {"F","R","B","L": cm}, "alt": m, "heading": deg,
   "obstacle_summary": {"F","FR","R","BR","B","BL","L","FL": cm} (8 sectors, optional),
   "caps": {"max_speed": m/s, "max_climb": m/s, "max_yaw": deg/s, "max_accel": m/s^2},
   "dt": s}
"""
import math

# ── tuning constants (single source of truth) ──────────────────────────────────────────────
ARM_M          = 0.225          # rotor arm; min standoff = 2x
MIN_STANDOFF   = 2 * ARM_M      # 0.45 m
DECEL_MS2      = 2.5            # achievable braking
REACTION_S     = 0.30           # loop/reaction latency built into caution radius
DEFAULT_CAPS   = {"max_speed": 0.8, "max_climb": 0.4, "max_yaw": 60.0, "max_accel": 0.4}
_SECTOR_UNIT = {  # body unit (fwd, right) per 8-sector bearing
    "F": (1, 0), "FR": (0.707, 0.707), "R": (0, 1), "BR": (-0.707, 0.707),
    "B": (-1, 0), "BL": (-0.707, -0.707), "L": (0, -1), "FL": (0.707, -0.707)}
_SECTOR_BEARING = {"F": 0, "FR": 45, "R": 90, "BR": 135, "B": 180, "BL": 225, "L": 270, "FL": 315}


# The pilot path (local_er_brain._speed_cap_cm / director_core._clearance_speed_cap) caps speed
# with a conservative LOOKUP TABLE. This module derived it from physics instead, which is sound
# but markedly more permissive -- at 80 cm clearance the table allows 0.20 m/s while the formula
# allowed 0.57 m/s. Same obstacle, same drone, three times the speed depending only on which code
# path was driving. The table is the binding limit everywhere now; the formula may only be
# stricter, never looser.
def _table_cap_ms(cm):
    """The conservative clearance table shared with the pilot path. Keep in sync with
    local_er_brain._speed_cap_cm and director_core._clearance_speed_cap."""
    if cm < 60:
        return 0.0
    if cm < 100:
        return 0.20
    if cm < 150:
        return 0.35
    if cm < 250:
        return 0.50
    return 0.60


def speed_for_clearance(cm, max_speed):
    """Max safe speed (m/s) toward a direction with `cm` clearance — the stopping-distance guarantee.
    Physics bound v <= sqrt(2*DECEL*(clear - standoff)) minus reaction creep, then held to the
    shared conservative table and the airframe limit."""
    m = cm / 100.0
    usable = m - MIN_STANDOFF
    if usable <= 0:
        return 0.0
    v = math.sqrt(2 * DECEL_MS2 * usable) - REACTION_S * DECEL_MS2  # subtract reaction creep
    return max(0.0, min(v, max_speed, _table_cap_ms(cm)))


def _ease(prev, target, max_step):
    """Jerk-limit: move `prev` toward `target` by at most `max_step` (ease-in/out, no step inputs)."""
    d = target - prev
    if d > max_step:  d = max_step
    if d < -max_step: d = -max_step
    return prev + d


def _bearing_clearance(ctx, bearing_deg):
    """Clearance (cm) toward a body bearing, from ToF (cardinal) or nearest sector."""
    tof = ctx.get("tof") or {}
    b = bearing_deg % 360
    card = {0: "F", 90: "R", 180: "B", 270: "L"}
    if round(b) in card and card[round(b)] in tof and tof[card[round(b)]] is not None:
        return tof[card[round(b)]]
    summ = ctx.get("obstacle_summary") or {}
    if summ:  # nearest sector to this bearing
        best = min(summ.keys(), key=lambda s: abs(((_SECTOR_BEARING[s] - b + 180) % 360) - 180))
        return summ.get(best, 9999)
    return 9999


def resolve_intent(intent, ctx, prev_v):
    """INTENT + live CTX -> precise, eased (vx, vy, vz, yaw_rate). Deterministic, never guessed."""
    caps = {**DEFAULT_CAPS, **(ctx.get("caps") or {})}
    dt = ctx.get("dt", 0.1)
    pvx, pvy, pvz, pyaw = prev_v
    man = str(intent.get("maneuver", "HOLD")).upper()
    pace = 0.6 if str(intent.get("pace", "norm")) == "slow" else 1.0
    conf = float(intent.get("conf", 1.0) or 0.0)
    if conf < 0.5:  # low confidence -> go gentle
        pace *= 0.6
    vx = vy = vz = yaw = 0.0
    alt = ctx.get("alt", 0.0)

    if man == "ASCEND" or man == "DESCEND":
        tgt = intent.get("target_alt_m", alt + (0.5 if man == "ASCEND" else -0.5))
        err = tgt - alt
        vz = max(-caps["max_climb"], min(caps["max_climb"], err * 1.0)) * pace

    elif man == "SCAN":
        yd = str(intent.get("yaw_dir", "R")).upper()
        yaw = (-caps["max_yaw"] if yd == "L" else caps["max_yaw"] if yd == "R" else 0.0) * pace

    elif man == "GOTO":
        bdeg = float(intent.get("bearing_deg", 0.0))
        clr = _bearing_clearance(ctx, bdeg)
        sp = speed_for_clearance(clr, caps["max_speed"]) * pace
        fwd, rgt = math.cos(math.radians(bdeg)), math.sin(math.radians(bdeg))
        vx, vy = fwd * sp, rgt * sp

    elif man == "FOLLOW":
        tb = float(intent.get("target_bearing_deg", 0.0))
        tdist = intent.get("target_dist_m", 3.0)
        cur = (intent.get("subject_dist_m") or ctx.get("subject_dist_m") or tdist)
        err = cur - tdist                      # +ve: too far -> approach
        clr = _bearing_clearance(ctx, tb)
        sp = min(speed_for_clearance(clr, caps["max_speed"]), abs(err) * 0.8) * pace
        sgn = 1.0 if err > 0 else -1.0
        fwd, rgt = math.cos(math.radians(tb)), math.sin(math.radians(tb))
        vx, vy = fwd * sp * sgn, rgt * sp * sgn
        yaw = max(-caps["max_yaw"], min(caps["max_yaw"], tb * 1.5))   # keep subject ahead

    elif man == "ORBIT":
        tb = float(intent.get("target_bearing_deg", 90.0))   # subject off to a side
        radius = intent.get("target_dist_m", 3.0)
        cur = (intent.get("subject_dist_m") or ctx.get("subject_dist_m") or radius)
        # tangential (perpendicular to subject) + radial correction to hold radius
        tang_b = tb + 90.0
        clr = _bearing_clearance(ctx, tang_b)
        tang = speed_for_clearance(clr, caps["max_speed"]) * 0.6 * pace
        radial = max(-0.3, min(0.3, (cur - radius) * 0.5))   # +ve too far -> move toward subject
        fb, rb = math.cos(math.radians(tang_b)), math.sin(math.radians(tang_b))
        fr, rr = math.cos(math.radians(tb)), math.sin(math.radians(tb))
        vx = fb * tang + fr * radial
        vy = rb * tang + rr * radial
        yaw = max(-caps["max_yaw"], min(caps["max_yaw"], tb * 1.5))   # camera stays on subject

    elif man == "WALL_FOLLOW":
        side = str(intent.get("side", "R")).upper()
        want = intent.get("target_dist_m", 0.6) * 100.0   # desired wall clearance cm
        tof = ctx.get("tof") or {}
        front = tof.get("F", 9999)
        sidecm = tof.get(side, 9999)
        if front <= want + 20:                  # corner -> turn away from the wall
            yaw = (caps["max_yaw"] if side == "R" else -caps["max_yaw"]) * pace
        else:
            sp = speed_for_clearance(front, caps["max_speed"]) * pace
            vx = sp
            corr = max(-0.2, min(0.2, (sidecm - want) / 100.0 * 0.5))  # hold wall distance
            vy = corr if side == "R" else -corr

    # ── ease (accel-limit) everything; clamp to caps ───────────────────────────────────────
    ms = caps["max_accel"]
    vx = _ease(pvx, vx, ms); vy = _ease(pvy, vy, ms); vz = _ease(pvz, vz, caps["max_climb"])
    sp = math.hypot(vx, vy)
    if sp > caps["max_speed"]:
        vx *= caps["max_speed"] / sp; vy *= caps["max_speed"] / sp
    yaw = max(-caps["max_yaw"], min(caps["max_yaw"], yaw))
    return vx, vy, vz, yaw


def local_avoid(vx, vy, vz, ctx, state):
    """Deterministic safety clamp: brake the component heading into a near sector + dodge-repulsion
    away from near/incoming sectors. vz never blocked by horizontal sensors. (Onboard ArduPilot OA is
    the fast reflex; this is the laptop-side guarantee.) Returns safe (vx, vy, vz)."""
    summ = ctx.get("obstacle_summary") or {}
    if not summ:
        return vx, vy, vz
    caps = {**DEFAULT_CAPS, **(ctx.get("caps") or {})}
    speed = math.hypot(vx, vy)
    caution = max(MIN_STANDOFF, min(MIN_STANDOFF + speed * REACTION_S + speed * speed / (2 * DECEL_MS2), 2.5))
    now = ctx.get("t", 0.0)
    prev = state.get("_avoid_prev", {})
    dt = max(1e-3, now - state.get("_avoid_prev_t", now))
    bf, br = vx, vy
    rf = rr = 0.0
    for d, (uf, ur) in _SECTOR_UNIT.items():
        cm = summ.get(d)
        if cm is None:
            continue
        dm = cm / 100.0
        if dm >= caution:
            continue
        intr = (caution - dm) / caution
        closing = ((prev.get(d, cm) - cm) / 100.0) / dt
        boost = 1.0 + max(0.0, closing) * 1.5
        rf -= uf * 0.55 * intr * boost
        rr -= ur * 0.55 * intr * boost
        toward = bf * uf + br * ur
        if toward > 0:
            bf -= uf * toward * intr
            br -= ur * toward * intr
    state["_avoid_prev"] = dict(summ)
    state["_avoid_prev_t"] = now
    sf, sr = bf + rf, br + rr
    mag = math.hypot(sf, sr)
    if mag > caps["max_speed"]:
        sf *= caps["max_speed"] / mag; sr *= caps["max_speed"] / mag
    return sf, sr, vz


class Step:
    """One sub-goal in the mission plan.
    intent: the maneuver dict to run while on this step.
    done(ctx, state) -> bool : measurable completion test (deterministic).
    subgoal_text: short description handed to Qwen for semantic confirmation (optional)."""
    def __init__(self, name, intent, done, subgoal_text=""):
        self.name = name
        self.intent = intent
        self.done = done
        self.subgoal_text = subgoal_text or name


class MissionSequencer:
    """Tracks the Gemini plan: which step we're on, advances when a step's predicate is met.
    This is what makes complex multi-step missions reliable — the AI never self-sequences."""
    def __init__(self, steps):
        self.steps = list(steps)
        self.i = 0
        self.state = {}            # scratch for predicates/avoidance (corner counts, prev sensors…)
        self.finished = False

    def current(self):
        if self.i >= len(self.steps):
            self.finished = True
            return None
        return self.steps[self.i]

    def tick(self, ctx, prev_v):
        """Advance step if done, then resolve the current step's intent into a safe velocity.
        Returns (vx, vy, vz, yaw_rate, step_name)."""
        # corner counter for WALL_FOLLOW (accumulate yaw the resolver commands)
        self._track(ctx, prev_v)
        step = self.current()
        if step is None:
            return 0.0, 0.0, 0.0, 0.0, "DONE"
        if step.done(ctx, self.state):
            self.i += 1
            self.state["step_entered_t"] = ctx.get("t", 0.0)
            step = self.current()
            if step is None:
                return 0.0, 0.0, 0.0, 0.0, "DONE"
        intent = dict(step.intent)
        intent.setdefault("conf", 1.0)
        vx, vy, vz, yaw = resolve_intent(intent, ctx, prev_v)
        vx, vy, vz = local_avoid(vx, vy, vz, ctx, self.state)
        self.state["_last_yaw"] = yaw
        return vx, vy, vz, yaw, step.name

    def _track(self, ctx, prev_v):
        # accumulate commanded yaw to count corners (used by WALL_FOLLOW done predicates)
        yaw = self.state.get("_last_yaw", 0.0)
        self.state["yaw_accum"] = self.state.get("yaw_accum", 0.0) + abs(yaw) * ctx.get("dt", 0.1)
        self.state["corners"] = int(self.state["yaw_accum"] / 80.0)
