from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Tuple

from .config import AppConfig
from .track import line_y_at, track_heading_at
from .utils import clamp, ease_towards


@dataclass
class PIDGains:
    kp: float
    ki: float
    kd: float
    trim: float


@dataclass
class ControllerState:
    I_state: float = 0.0
    e_prev: float = 0.0
    u: float = 0.0
    u_prev: float = 0.0
    ctrl_accum: float = 0.0


@dataclass
class RobotState:
    xw: float
    yw: float
    heading: float
    phase: float = 0.0

    pwm_l_t: float = 0.0
    pwm_r_t: float = 0.0
    pwm_l: float = 0.0
    pwm_r: float = 0.0


def body_error(cfg: AppConfig, xw: float, yw: float, heading: float, phase: float) -> Tuple[float, float, float]:
    """Error at front and rear of the body relative to track centerline."""
    c = math.cos(heading)
    s = math.sin(heading)
    fx = xw + cfg.robot.body_front_offset * c
    fy = yw + cfg.robot.body_front_offset * s
    rx = xw + cfg.robot.body_rear_offset * c
    ry = yw + cfg.robot.body_rear_offset * s
    eF = line_y_at(fx, phase, cfg.track) - fy
    eR = line_y_at(rx, phase, cfg.track) - ry
    return eF, eR, 0.5 * (eF + eR)


def controller_error_front(cfg: AppConfig, xw: float, yw: float, heading: float, phase: float) -> Tuple[float, float, float, float]:
    """Lookahead tracking error at a point in front of the robot."""
    c = math.cos(heading)
    s = math.sin(heading)
    la_xw = xw + cfg.robot.lookahead * c
    la_yw = yw + cfg.robot.lookahead * s
    y_line = line_y_at(la_xw, phase, cfg.track)
    return y_line - la_yw, la_xw, la_yw, y_line


def init_world(cfg: AppConfig, base_pwm: float) -> tuple[RobotState, ControllerState]:
    phase = 0.0
    xw = 140.0
    yw = line_y_at(xw, phase, cfg.track)
    heading = track_heading_at(xw, phase, cfg.track)

    rs = RobotState(
        xw=xw,
        yw=yw,
        heading=heading,
        phase=phase,
        pwm_l_t=base_pwm,
        pwm_r_t=base_pwm,
        pwm_l=base_pwm,
        pwm_r=base_pwm,
    )
    cs = ControllerState()
    return rs, cs


def step_sim(
    cfg: AppConfig,
    rs: RobotState,
    cs: ControllerState,
    gains: PIDGains,
    base_pwm: float,
    dt: float,
    *,
    paused: bool,
    speed_scale: float = 1.0,
    measure_noise: bool = True,
) -> dict:
    """
    One frame update:
    - compute measured error
    - update controller at DT_CTRL
    - update robot kinematics
    Returns measurements for charts / UI.
    """
    # Measurements (before physics, matching your original structure)
    e_front_true, la_xw, la_yw, y_line = controller_error_front(cfg, rs.xw, rs.yw, rs.heading, rs.phase)
    e_front_meas = e_front_true + (random.gauss(0.0, cfg.robot.sensor_noise_std) if measure_noise else 0.0)
    _, _, e_body = body_error(cfg, rs.xw, rs.yw, rs.heading, rs.phase)

    if not paused:
        rs.phase += cfg.track.phase_speed * dt

        # controller tick(s)
        cs.ctrl_accum += dt
        while cs.ctrl_accum >= cfg.robot.dt_ctrl:
            cs.ctrl_accum -= cfg.robot.dt_ctrl

            cs.I_state += e_front_meas * cfg.robot.dt_ctrl
            cs.I_state = clamp(cs.I_state, -5000.0, 5000.0)

            d_term = (e_front_meas - cs.e_prev) / cfg.robot.dt_ctrl
            cs.e_prev = e_front_meas

            cs.u_prev = cs.u
            cs.u = gains.kp * e_front_meas + gains.ki * cs.I_state + gains.kd * d_term

            base_eff = base_pwm * speed_scale
            rs.pwm_l_t = clamp(base_eff - cs.u + gains.trim, 0, 255)
            rs.pwm_r_t = clamp(base_eff + cs.u - gains.trim, 0, 255)

        # actuator lag
        rs.pwm_l = ease_towards(rs.pwm_l, rs.pwm_l_t, dt, tau=cfg.robot.actuator_lag_tau)
        rs.pwm_r = ease_towards(rs.pwm_r, rs.pwm_r_t, dt, tau=cfg.robot.actuator_lag_tau)

        # kinematics (differential drive)
        v_l = (rs.pwm_l / 255.0) * cfg.robot.max_speed_px
        v_r = (rs.pwm_r / 255.0) * cfg.robot.max_speed_px
        v_l *= (1.0 + cfg.robot.steering_bias)
        v_r *= (1.0 - cfg.robot.steering_bias)

        v = 0.5 * (v_l + v_r)
        omega = (v_r - v_l) / cfg.robot.wheel_base

        rs.heading += omega * dt
        rs.xw += v * math.cos(rs.heading) * dt
        rs.yw += v * math.sin(rs.heading) * dt
        rs.yw = clamp(rs.yw, 12, cfg.ui.height - 12)

        # safety pull-back
        if cfg.robot.soft_pull_to_line and abs(e_body) > cfg.robot.pull_threshold:
            y_target = line_y_at(rs.xw, rs.phase, cfg.track)
            rs.yw += (y_target - rs.yw) * clamp(cfg.robot.pull_strength * dt, 0.0, 1.0)

        # world shift
        if rs.xw > cfg.robot.world_shift_at:
            rs.xw -= cfg.robot.world_shift_by
            rs.phase += cfg.robot.world_shift_by  # consistent with your original approach

    return {
        "e_front_true": e_front_true,
        "e_front_meas": e_front_meas,
        "e_body": e_body,
        "la_xw": la_xw,
        "la_yw": la_yw,
        "y_line": y_line,
    }