from __future__ import annotations

"""
Webots controller inspired by the Emmy article, but adapted conservatively for this robot model.

What this version adds on top of the already stable anti-collision controller:
- Uses the 12-state Para-Analisador partition in the article (V, F, T, P and 8 non-extreme states).
- Shows, in debug, both the logical state and the article routine associated with that state.
- Executes article-style routines with conservative timing, while keeping ESCAPE as the highest-priority safety layer.

Important adaptation note:
- The article reports routine structures and reference times T1/T2/T3.
- In this Webots model, the same routine STRUCTURE is preserved, but forward-motion times are scaled down
  so the robot keeps the good "not hitting anything" behavior you already achieved.
"""

from dataclasses import dataclass
from typing import Deque, List, Tuple
from collections import deque
import math

from controller import Robot, Motor, DistanceSensor


@dataclass(frozen=True)
class Cfg:
    # --- Sensor distance mapping (m) ---
    d_near: float = 0.18
    d_far: float = 1.50

    # --- Evidence shaping (μ and λ) ---
    mu_near: float = 0.18
    mu_far: float = 0.75
    mu_gamma: float = 1.60

    lam_near: float = 0.25
    lam_far: float = 0.85
    lam_gamma: float = 0.90

    mu_ema_alpha: float = 0.08
    lam_ema_alpha: float = 0.08
    mu_slew_per_s: float = 0.40
    lam_slew_per_s: float = 0.40

    # Evidence mode:
    # article_front:
    #   proposition p := "Existe obstáculo à frente"
    #   μ  is strengthened by the closer sensor (obstacle evidence),
    #   λ  is strengthened by the farther sensor (free-path / contrary evidence).
    # This preserves the article semantics while still using the two lateral sensors
    # of this Webots model to generate contradiction / indefinition zones.
    #
    # article_turn_legacy:
    #   previous experimental mode kept only for backward compatibility.
    evidence_mode: str = "article_front"

    # Article thresholds (C1, C2, C3, C4)
    c1_gc_upper_true: float = 0.55
    c2_gc_lower_false: float = -0.55
    c3_gct_upper_inconsistent: float = 0.55
    c4_gct_lower_paracomplete: float = -0.55

    # --- Avoidance hysteresis ---
    avoid_enter: float = 0.11
    avoid_exit: float = 0.15

    # --- Steering / keep-away ---
    d_target: float = 0.10
    k_rep: float = 0.95
    err_deadband_m: float = 0.06
    headon_enter: float = 0.15
    min_turn_close: float = 0.26
    min_turn_headon: float = 0.16
    k_err: float = 4.6
    max_turn: float = 0.75
    turn_dcap: float = 0.65

    # --- Speeds ---
    cruise_frac: float = 0.45
    slow_frac: float = 0.22
    routine_fast_frac: float = 0.42
    routine_search_frac: float = 0.24
    routine_cautious_frac: float = 0.18

    # --- ESCAPE ---
    back_frac: float = 0.30
    back_curve_turn: float = 0.35
    pivot_spin_frac: float = 0.70
    exit_turn: float = 0.30
    back_min_s: float = 0.35
    back_max_s: float = 1.05
    pivot_min_s: float = 0.45
    pivot_max_s: float = 1.45
    exit_s: float = 0.70
    improve_eps: float = 0.03
    flip_check_s: float = 0.55

    # --- Wander ---
    wander_amp: float = 0.02
    wander_w: float = 0.33

    # --- Stuck detection ---
    stuck_window: int = 30
    stuck_span_m: float = 0.008
    stuck_close_m: float = 0.24
    stuck_detect_dmax: float = 0.34
    stuck_pair_span_m: float = 0.010
    avoid_min_time_s: float = 1.6

    # --- Filtering ---
    ema_alpha: float = 0.20

    # --- Cooldown after escape ---
    escape_cooldown_s: float = 0.90
    escape_cooldown_turn: float = 0.22

    # --- Impact/contact zone ---
    impact_enabled: bool = True
    impact_distance: float = 0.10
    impact_time_s: float = 0.06
    impact_react: bool = False

    # --- Article routine execution (adapted conservatively) ---
    # Article reference values: T1=0.75s, T2=1.5s, T3=2.0s.
    article_t1_s: float = 0.75
    article_t2_s: float = 1.50
    article_t3_s: float = 2.00
    article_time_scale: float = 0.30   # conservative scaling for this Webots model
    article_strict_timing: bool = False # if True, uses the literal article times T1/T2/T3

    turn45_s: float = 0.16             # calibrated for this robot model
    turn90_s: float = 0.30
    signal_hold_s: float = 0.25
    state_stable_steps: int = 4
    retrigger_cooldown_s: float = 0.10

    debug: bool = True
    debug_show_nominal_exec: bool = True


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else 1.0 if x > 1.0 else x


def evidence_obstacle(d: float, cfg: Cfg) -> float:
    near = cfg.mu_near
    far = max(cfg.mu_far, near + 1e-6)
    if d <= near:
        x = 1.0
    elif d >= far:
        x = 0.0
    else:
        x = (far - d) / (far - near)
    return clamp01(x) ** (cfg.mu_gamma if cfg.mu_gamma > 0 else 1.0)


def evidence_close_lambda(d: float, cfg: Cfg) -> float:
    near = cfg.lam_near
    far = max(cfg.lam_far, near + 1e-6)
    if d <= near:
        x = 1.0
    elif d >= far:
        x = 0.0
    else:
        x = (far - d) / (far - near)
    return clamp01(x) ** (cfg.lam_gamma if cfg.lam_gamma > 0 else 1.0)


def evidence_free(d: float, cfg: Cfg) -> float:
    near = cfg.lam_near
    far = max(cfg.lam_far, near + 1e-6)
    if d <= near:
        x = 0.0
    elif d >= far:
        x = 1.0
    else:
        x = (d - near) / (far - near)
    return clamp01(x) ** (cfg.lam_gamma if cfg.lam_gamma > 0 else 1.0)


def compute_article_evidence(d_left: float, d_right: float, cfg: Cfg) -> Tuple[float, float]:
    """Return (μ, λ) with article semantics for proposition p = 'Existe obstáculo à frente'.

    Adaptation used in this Webots model:
    - μ (favorable evidence) is reinforced by the closer sensor, i.e., obstacle evidence.
    - λ (contrary evidence) is reinforced by the farther sensor, i.e., free-path evidence.

    This gives the intended LPA2v geometry:
    - both near  -> μ high, λ low  -> V
    - both far   -> μ low,  λ high -> F
    - one near / one far -> μ high and λ high -> T
    - both medium / weak -> μ low and λ low -> P
    """
    dmin = min(d_left, d_right)
    dmax = max(d_left, d_right)

    if cfg.evidence_mode == "article_turn_legacy":
        mu_raw = evidence_obstacle(d_left, cfg)
        lam_raw = evidence_close_lambda(d_right, cfg)
        return mu_raw, lam_raw

    mu_raw = evidence_obstacle(dmin, cfg)
    lam_raw = evidence_free(dmax, cfg)
    return mu_raw, lam_raw


def state_supports_obstacle_hypothesis(state: str) -> bool:
    return state in ("V", "T", "QV_T", "T_V", "QF_T", "T_F")


def para_analyzer_article(mu: float, lam: float, cfg: Cfg) -> Tuple[str, float, float]:
    """Implements the 12-state article partition.

    State codes:
      V, F, T, P,
      QV_T, T_V, QV_P, P_V,
      QF_P, P_F, QF_T, T_F
    """
    gc = mu - lam
    gct = mu + lam - 1.0

    c1 = cfg.c1_gc_upper_true
    c2 = cfg.c2_gc_lower_false
    c3 = cfg.c3_gct_upper_inconsistent
    c4 = cfg.c4_gct_lower_paracomplete

    # Extreme states
    if gc >= c1:
        return "V", gc, gct
    if gc <= c2:
        return "F", gc, gct
    if gct >= c3:
        return "T", gc, gct
    if gct <= c4:
        return "P", gc, gct

    # Non-extreme states exactly as in the article logic
    if 0.0 <= gc < c1 and 0.0 <= gct < c3:
        return ("QV_T" if gc >= gct else "T_V"), gc, gct

    if 0.0 <= gc < c1 and c4 < gct <= 0.0:
        return ("QV_P" if gc >= abs(gct) else "P_V"), gc, gct

    if c2 < gc <= 0.0 and c4 < gct <= 0.0:
        return ("QF_P" if abs(gc) >= abs(gct) else "P_F"), gc, gct

    if c2 < gc <= 0.0 and 0.0 <= gct < c3:
        return ("QF_T" if abs(gc) >= gct else "T_F"), gc, gct

    # Fallback near boundaries
    if gc >= 0.0:
        return ("QV_T" if gct >= 0.0 else "QV_P"), gc, gct
    return ("QF_T" if gct >= 0.0 else "QF_P"), gc, gct


def seconds_to_steps(seconds: float, timestep_ms: int) -> int:
    return max(1, int(round(seconds * 1000.0 / max(1, timestep_ms))))


class EscapeSM:
    def __init__(self) -> None:
        self.mode = "OFF"
        self.dir = 1.0
        self.steps = 0
        self.min_steps = 0
        self.max_steps = 0
        self.exit_steps = 0
        self.dref = 0.0
        self.last_check_step = 0
        self.flipped_once = False

    def start(self, cfg: Cfg, timestep_ms: int, turn_dir: float, dmin: float, step: int) -> None:
        self.mode = "BACK"
        self.dir = turn_dir
        self.steps = 0
        self.min_steps = seconds_to_steps(cfg.back_min_s, timestep_ms)
        self.max_steps = seconds_to_steps(cfg.back_max_s, timestep_ms)
        self.exit_steps = seconds_to_steps(cfg.exit_s, timestep_ms)
        self.dref = dmin
        self.last_check_step = step
        self.flipped_once = False

    def active(self) -> bool:
        return self.mode != "OFF"

    def stop(self) -> None:
        self.mode = "OFF"
        self.steps = 0

    def update(self, cfg: Cfg, timestep_ms: int, dmin: float, err: float, head_on: bool, step: int) -> None:
        if self.mode == "OFF":
            return
        self.steps += 1

        flip_check_steps = seconds_to_steps(cfg.flip_check_s, timestep_ms)
        if self.mode == "PIVOT" and (not self.flipped_once) and (step - self.last_check_step) >= flip_check_steps:
            improved = dmin >= self.dref + cfg.improve_eps
            broke_headon = (not head_on) and (abs(err) >= cfg.err_deadband_m)
            if (not improved) and (not broke_headon):
                self.dir *= -1.0
                self.flipped_once = True
            self.dref = dmin
            self.last_check_step = step

        if self.mode == "BACK":
            if self.steps >= self.min_steps and dmin >= cfg.avoid_exit:
                self.mode = "PIVOT"
                self.steps = 0
                self.min_steps = seconds_to_steps(cfg.pivot_min_s, timestep_ms)
                self.max_steps = seconds_to_steps(cfg.pivot_max_s, timestep_ms)
                self.dref = dmin
                self.last_check_step = step
                return
            if self.steps >= self.max_steps:
                self.mode = "PIVOT"
                self.steps = 0
                self.min_steps = seconds_to_steps(cfg.pivot_min_s, timestep_ms)
                self.max_steps = seconds_to_steps(cfg.pivot_max_s, timestep_ms)
                self.dref = dmin
                self.last_check_step = step
                return

        elif self.mode == "PIVOT":
            if self.steps >= self.min_steps:
                if (not head_on) and dmin >= cfg.avoid_exit:
                    self.mode = "EXIT"
                    self.steps = 0
                    return
            if self.steps >= self.max_steps:
                self.mode = "EXIT"
                self.steps = 0
                return

        elif self.mode == "EXIT":
            if self.steps >= self.exit_steps:
                self.mode = "OFF"


class RoutineSM:
    def __init__(self) -> None:
        self.routine_code = "-"
        self.routine_name = "-"
        self.action_label = "-"
        self.steps_left = 0
        self.queue: List[Tuple[str, int]] = []
        self.signal_on = False
        self.last_start_step = -10**9

    def active(self) -> bool:
        return self.steps_left > 0 or bool(self.queue)

    def stop(self) -> None:
        self.routine_code = "-"
        self.routine_name = "-"
        self.action_label = "-"
        self.steps_left = 0
        self.queue = []
        self.signal_on = False

    def start(self, routine_code: str, routine_name: str, seq: List[Tuple[str, int]], step: int) -> None:
        self.routine_code = routine_code
        self.routine_name = routine_name
        self.queue = [(a, s) for a, s in seq if s > 0]
        self.steps_left = 0
        self.action_label = "-"
        self.signal_on = False
        self.last_start_step = step
        self._advance()

    def _advance(self) -> None:
        while self.queue:
            action, steps = self.queue.pop(0)
            self.action_label = action
            self.steps_left = steps
            if action == "SIGNAL_ON":
                self.signal_on = True
            elif action == "SIGNAL_OFF":
                self.signal_on = False
            else:
                return
        self.action_label = "-"
        self.steps_left = 0

    def tick(self) -> str:
        current = self.action_label
        if self.steps_left > 0:
            self.steps_left -= 1
            if self.steps_left <= 0:
                self._advance()
        return current


def drive_diff(left_motor: Motor, right_motor: Motor, left: float, right: float, vmax: float) -> None:
    left_motor.setVelocity(clamp(left, -vmax, vmax))
    right_motor.setVelocity(clamp(right, -vmax, vmax))


def article_state_label(state: str) -> str:
    labels = {
        "V": "V",
        "F": "F",
        "T": "T",
        "P": "⊥",
        "QV_T": "QV→T",
        "T_V": "T→V",
        "QV_P": "QV→⊥",
        "P_V": "⊥→V",
        "QF_P": "QF→⊥",
        "P_F": "⊥→F",
        "QF_T": "QF→T",
        "T_F": "T→F",
    }
    return labels.get(state, state)


def build_article_routine(state: str, turn_dir: float, cfg: Cfg, timestep_ms: int) -> Tuple[str, str, List[Tuple[str, int]]]:
    time_scale = 1.0 if cfg.article_strict_timing else cfg.article_time_scale
    t1 = seconds_to_steps(cfg.article_t1_s * time_scale, timestep_ms)
    t2 = seconds_to_steps(cfg.article_t2_s * time_scale, timestep_ms)
    t3 = seconds_to_steps(cfg.article_t3_s * time_scale, timestep_ms)
    tr45 = seconds_to_steps(cfg.turn45_s, timestep_ms)
    tr90 = seconds_to_steps(cfg.turn90_s, timestep_ms)
    sig = seconds_to_steps(cfg.signal_hold_s, timestep_ms)

    # Exact article mapping for non-extreme states.
    if state == "V":
        return "R1", "Desvio rápido do obstáculo", [("TURN_L90", tr90), ("FWD_T1", t1), ("TURN_R90", tr90), ("FWD_T2", t2)]
    if state == "F":
        return "R2", "Avanço rápido com confiança", [("FWD_T2", t2), ("FWD_T2", t2), ("FWD_T1", t1)]

    # For extreme ambiguous states, the article shows R3/R4 variants.
    if state == "T":
        if turn_dir >= 0.0:
            return "R4", "Procura novo ângulo (T, direita)", [("TURN_R90", tr90), ("FWD_T3", t3), ("TURN_L90", tr90)]
        return "R3", "Procura novo ângulo (T, esquerda)", [("TURN_L90", tr90), ("FWD_T1", t1), ("TURN_R90", tr90)]

    if state == "P":
        if turn_dir >= 0.0:
            return "R3", "Procura novo ângulo (⊥, direita)", [("TURN_R90", tr90), ("FWD_T1", t1), ("TURN_L90", tr90)]
        return "R4", "Procura novo ângulo (⊥, esquerda)", [("TURN_L90", tr90), ("FWD_T3", t3), ("TURN_R90", tr90)]

    if state == "QV_T":
        return "R6", "QV→T: desvio lento e para", [("TURN_L45", tr45), ("STOP", sig)]
    if state == "QF_T":
        return "R7", "QF→T: avanço cauteloso e para", [("FWD_T1", t1), ("FWD_T1", t1), ("STOP", sig)]
    if state == "QV_P":
        return "R8", "QV→⊥: desvio 45° à direita e para", [("TURN_R45", tr45), ("STOP", sig)]
    if state == "QF_P":
        return "R9", "QF→⊥: avanço T1 e para", [("FWD_T1", t1), ("STOP", sig)]
    if state == "T_V":
        return "R10", "T→V: desvio 45° à esquerda e para", [("TURN_L45", tr45), ("STOP", sig)]
    if state == "T_F":
        return "R11", "T→F: avanço T1 e para", [("FWD_T1", t1), ("STOP", sig)]
    if state == "P_V":
        return "R12", "⊥→V: desvio 45° à direita, para e sinaliza", [("TURN_R45", tr45), ("STOP", sig), ("SIGNAL_ON", sig), ("SIGNAL_OFF", 1)]
    if state == "P_F":
        return "R13", "⊥→F: avanço T1, para e sinaliza", [("FWD_T1", t1), ("STOP", sig), ("SIGNAL_ON", sig), ("SIGNAL_OFF", 1)]

    return "-", "-", []


def execute_routine_action(
    action: str,
    routine_code: str,
    vmax: float,
    v_fast: float,
    v_search: float,
    v_cautious: float,
) -> Tuple[float, float]:
    spin45 = 0.42 * vmax
    spin90 = 0.52 * vmax
    stop = 0.0

    if action in ("-", "STOP"):
        return stop, stop

    if action == "FWD_T1":
        if routine_code == "R2":
            v = v_fast
        elif routine_code in ("R1", "R3", "R4"):
            v = v_search
        else:
            v = v_cautious
        return v, v

    if action == "FWD_T2":
        v = v_fast if routine_code in ("R1", "R2") else v_search
        return v, v

    if action == "FWD_T3":
        return v_search, v_search

    if action == "TURN_L45":
        return -spin45, +spin45
    if action == "TURN_R45":
        return +spin45, -spin45
    if action == "TURN_L90":
        return -spin90, +spin90
    if action == "TURN_R90":
        return +spin90, -spin90
    return stop, stop


def main() -> None:
    robot = Robot()
    timestep_ms = int(robot.getBasicTimeStep())
    cfg = Cfg(debug=True)
    print('>>> Controller v15a (LPA2v + rotinas do artigo + segurança conservadora) carregado | debug=%s <<<' % cfg.debug, flush=True)

    right_motor: Motor = robot.getDevice("motor_1")
    left_motor: Motor = robot.getDevice("motor_2")
    left_motor.setPosition(float("inf"))
    right_motor.setPosition(float("inf"))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    ds_left: DistanceSensor = robot.getDevice("ds_left")
    ds_right: DistanceSensor = robot.getDevice("ds_right")
    ds_left.enable(timestep_ms)
    ds_right.enable(timestep_ms)

    max_speed_signed = -6.28   # forward is negative in this model
    vmax = abs(max_speed_signed)
    v_cruise = -cfg.cruise_frac * vmax
    v_slow = -cfg.slow_frac * vmax
    v_fast = -cfg.routine_fast_frac * vmax
    v_search = -cfg.routine_search_frac * vmax
    v_cautious = -cfg.routine_cautious_frac * vmax
    v_back = +cfg.back_frac * vmax
    spin_escape = cfg.pivot_spin_frac * vmax

    dL_f: float | None = None
    dR_f: float | None = None
    mu_f: float | None = None
    lam_f: float | None = None

    avoid_mode = False
    turn_dir = 1.0

    escape = EscapeSM()
    routine = RoutineSM()
    cooldown_steps = 0
    cooldown_dir = 1.0

    impact_count = 0
    impact_required_steps = seconds_to_steps(cfg.impact_time_s, timestep_ms)
    retrigger_steps = seconds_to_steps(cfg.retrigger_cooldown_s, timestep_ms)

    dmin_hist: Deque[float] = deque(maxlen=cfg.stuck_window)
    pair_hist: Deque[Tuple[float, float]] = deque(maxlen=cfg.stuck_window)
    avoid_steps = 0

    last_state = ""
    stable_count = 0
    warmup_steps = 10
    step = 0

    while robot.step(timestep_ms) != -1:
        step += 1

        dL = float(ds_left.getValue())
        dR = float(ds_right.getValue())

        if dL_f is None:
            dL_f, dR_f = dL, dR
        else:
            a = cfg.ema_alpha
            dL_f = a * dL + (1.0 - a) * dL_f
            dR_f = a * dR + (1.0 - a) * dR_f

        assert dL_f is not None and dR_f is not None

        dmin = min(dL_f, dR_f)
        if cfg.impact_enabled and dmin <= cfg.impact_distance:
            impact_count += 1
        else:
            impact_count = 0
        impact = cfg.impact_enabled and impact_count >= impact_required_steps

        dL_t = min(dL_f, cfg.turn_dcap)
        dR_t = min(dR_f, cfg.turn_dcap)
        err = dR_t - dL_t
        head_on = abs(err) <= cfg.err_deadband_m

        if abs(err) >= (cfg.err_deadband_m * 1.8):
            turn_dir = 1.0 if err > 0 else -1.0
        else:
            if dL_f < dR_f - 1e-6:
                turn_dir = 1.0
            elif dR_f < dL_f - 1e-6:
                turn_dir = -1.0

        if step <= warmup_steps:
            drive_diff(left_motor, right_motor, v_slow, v_slow, vmax)
            continue

        mu_raw, lam_raw = compute_article_evidence(dL_f, dR_f, cfg)

        mu_prev = mu_f if mu_f is not None else mu_raw
        lam_prev = lam_f if lam_f is not None else lam_raw

        if mu_f is None:
            mu_f, lam_f = mu_raw, lam_raw
        else:
            mu_f = cfg.mu_ema_alpha * mu_raw + (1.0 - cfg.mu_ema_alpha) * mu_f
            lam_f = cfg.lam_ema_alpha * lam_raw + (1.0 - cfg.lam_ema_alpha) * lam_f

        dt = timestep_ms / 1000.0
        dmu_max = cfg.mu_slew_per_s * dt
        dlam_max = cfg.lam_slew_per_s * dt

        dmu = mu_f - mu_prev
        if dmu > dmu_max:
            mu_f = mu_prev + dmu_max
        elif dmu < -dmu_max:
            mu_f = mu_prev - dmu_max

        dlam = lam_f - lam_prev
        if dlam > dlam_max:
            lam_f = lam_prev + dlam_max
        elif dlam < -dlam_max:
            lam_f = lam_prev - dlam_max

        mu = clamp01(mu_f)
        lam = clamp01(lam_f)
        state, gc, gct = para_analyzer_article(mu, lam, cfg)

        if state == last_state:
            stable_count += 1
        else:
            stable_count = 1
            last_state = state

        routine_code_nom, routine_name_nom, routine_seq_nom = build_article_routine(state, turn_dir, cfg, timestep_ms)

        logic_obstacle = state_supports_obstacle_hypothesis(state)
        if not avoid_mode and (dmin <= cfg.avoid_enter or logic_obstacle):
            avoid_mode = True
        elif avoid_mode and (dmin >= cfg.avoid_exit and (not logic_obstacle)):
            avoid_mode = False

        if avoid_mode:
            avoid_steps += 1
        else:
            avoid_steps = 0

        dmin_hist.append(dmin)
        pair_hist.append((dL_f, dR_f))
        stuck = False
        if avoid_steps >= seconds_to_steps(cfg.avoid_min_time_s, timestep_ms) and len(pair_hist) == pair_hist.maxlen:
            dL_vals = [p[0] for p in pair_hist]
            dR_vals = [p[1] for p in pair_hist]
            spanL = max(dL_vals) - min(dL_vals)
            spanR = max(dR_vals) - min(dR_vals)
            if dmin <= cfg.stuck_detect_dmax and spanL <= cfg.stuck_pair_span_m and spanR <= cfg.stuck_pair_span_m:
                stuck = True
        if (not stuck) and len(dmin_hist) == dmin_hist.maxlen:
            span = max(dmin_hist) - min(dmin_hist)
            if span <= cfg.stuck_span_m and (sum(dmin_hist) / len(dmin_hist) <= cfg.stuck_close_m):
                stuck = True

        if (not escape.active()) and (dmin <= cfg.avoid_enter * 0.55 or stuck or (cfg.impact_react and impact)):
            esc_dir = turn_dir if head_on else (1.0 if err > 0 else -1.0)
            escape.start(cfg, timestep_ms, esc_dir, dmin, step)
            cooldown_dir = esc_dir
            routine.stop()

        if escape.active():
            escape.update(cfg, timestep_ms, dmin, err, head_on, step)

        if (not escape.active()) and cooldown_steps == 0 and dmin <= cfg.avoid_exit and step > warmup_steps:
            cooldown_steps = seconds_to_steps(cfg.escape_cooldown_s, timestep_ms)

        # Start the article routine only when the state is stable for a few cycles.
        if (not escape.active()) and cooldown_steps == 0 and (not routine.active()):
            enough_time_since_last = (step - routine.last_start_step) >= retrigger_steps
            if stable_count >= cfg.state_stable_steps and routine_code_nom != "-" and enough_time_since_last:
                routine.start(routine_code_nom, routine_name_nom, routine_seq_nom, step)

        active_action = routine.action_label if routine.active() else "-"
        exec_code = routine.routine_code if routine.active() else "-"
        exec_name = routine.routine_name if routine.active() else "-"

        if cfg.debug and (step % 8 == 0):
            if cfg.debug_show_nominal_exec:
                print(
                    f"L={dL_f:.3f} R={dR_f:.3f} min={dmin:.3f} err={err:+.3f} head={head_on} "
                    f"mu={mu:.2f} lam={lam:.2f} state={article_state_label(state):<5} "
                    f"Gc={gc:+.2f} Gct={gct:+.2f} rotNom={routine_code_nom:<4} rotExec={exec_code:<4} "
                    f"nom=({routine_name_nom}) exec=({exec_name}) "
                    f"act={active_action:<10} avoid={avoid_mode} stuck={stuck} esc={escape.mode:<5} sig={'ON' if routine.signal_on else 'OFF'}"
                , flush=True)
            else:
                print(
                    f"L={dL_f:.3f} R={dR_f:.3f} min={dmin:.3f} err={err:+.3f} head={head_on} "
                    f"mu={mu:.2f} lam={lam:.2f} state={article_state_label(state):<5} "
                    f"Gc={gc:+.2f} Gct={gct:+.2f} rot={routine_code_nom:<4} ({routine_name_nom}) "
                    f"act={active_action:<10} avoid={avoid_mode} stuck={stuck} esc={escape.mode:<5} sig={'ON' if routine.signal_on else 'OFF'}"
                , flush=True)

        # --- CONTROL PRIORITY ---
        if escape.active():
            if escape.mode == "BACK":
                turn = -cfg.back_curve_turn * escape.dir
                left = v_back * (1.0 + turn)
                right = v_back * (1.0 - turn)
                drive_diff(left_motor, right_motor, left, right, vmax)
                continue
            if escape.mode == "PIVOT":
                drive_diff(left_motor, right_motor, +escape.dir * spin_escape, -escape.dir * spin_escape, vmax)
                continue
            if escape.mode == "EXIT":
                turn = cfg.exit_turn * escape.dir
                left = v_slow * (1.0 + turn)
                right = v_slow * (1.0 - turn)
                drive_diff(left_motor, right_motor, left, right, vmax)
                continue

        if cooldown_steps > 0:
            cooldown_steps -= 1
            turn = cfg.escape_cooldown_turn * cooldown_dir
            left = v_slow * (1.0 + turn)
            right = v_slow * (1.0 - turn)
            drive_diff(left_motor, right_motor, left, right, vmax)
            continue

        if routine.active():
            current_action = routine.tick()
            left, right = execute_routine_action(
                current_action,
                routine.routine_code,
                vmax,
                v_fast,
                v_search,
                v_cautious,
            )
            drive_diff(left_motor, right_motor, left, right, vmax)
            continue

        # --- Fallback continuous control (keeps the good no-hit behavior between routines) ---
        if avoid_mode or dmin <= cfg.avoid_enter:
            base = v_slow
        else:
            t = clamp01((dmin - cfg.avoid_enter) / max(1e-6, (cfg.d_far - cfg.avoid_enter)))
            base = (1.0 - t) * v_slow + t * v_cruise

        turn = cfg.k_err * err
        repL = clamp((cfg.d_target - dL_f) / max(1e-6, cfg.d_target), 0.0, 2.0)
        repR = clamp((cfg.d_target - dR_f) / max(1e-6, cfg.d_target), 0.0, 2.0)
        turn += cfg.k_rep * repL
        turn -= cfg.k_rep * repR
        turn = clamp(turn, -cfg.max_turn, cfg.max_turn)
        if avoid_mode:
            turn = clamp(turn, -0.60, 0.60)

        if head_on and dmin <= cfg.headon_enter:
            turn = turn_dir * max(cfg.min_turn_headon, abs(turn))
        elif dmin <= cfg.avoid_enter and abs(turn) < cfg.min_turn_close:
            if dL_f < dR_f - 1e-6:
                sign = +1.0
            elif dR_f < dL_f - 1e-6:
                sign = -1.0
            else:
                sign = turn_dir
            turn = sign * cfg.min_turn_close

        if (not avoid_mode) and (dmin > 0.90):
            turn += cfg.wander_amp * math.sin(cfg.wander_w * robot.getTime())

        turn = clamp(turn, -cfg.max_turn, cfg.max_turn)
        left = base * (1.0 + turn)
        right = base * (1.0 - turn)
        drive_diff(left_motor, right_motor, left, right, vmax)


if __name__ == "__main__":
    main()
