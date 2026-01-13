from __future__ import annotations

import math
from dataclasses import asdict
from typing import Dict

from .config import AppConfig
from .sim import PIDGains, RobotState, ControllerState, body_error, controller_error_front
from .track import line_y_at
from .utils import clamp, ease_towards


def _snapshot_to_state(cfg: AppConfig, snap: Dict) -> tuple[RobotState, ControllerState]:
    rs = RobotState(
        xw=snap["xw"],
        yw=snap["yw"],
        heading=snap["heading"],
        phase=snap["phase"],
        pwm_l=snap["pwm_l"],
        pwm_r=snap["pwm_r"],
        pwm_l_t=snap.get("pwm_l_t", snap["pwm_l"]),
        pwm_r_t=snap.get("pwm_r_t", snap["pwm_r"]),
    )
    cs = ControllerState(
        I_state=snap["I_state"],
        e_prev=snap["e_prev"],
        ctrl_accum=snap["ctrl_accum"],
        u_prev=snap["u_prev"],
        u=snap["u"],
    )
    return rs, cs


class ShadowSimulator:
    """
    Fast "shadow" rollout used by the auto-tuner.
    Mirrors the real sim dynamics closely (same controller update cadence).
    """

    def __init__(self, cfg: AppConfig):
        self.cfg = cfg

    def quick_cost(self, snap: Dict, gains_dict: Dict, base_pwm: float, speed_scale: float) -> float:
        cfg = self.cfg
        rs, cs = _snapshot_to_state(cfg, snap)
        gains = PIDGains(kp=gains_dict["kp"], ki=gains_dict["ki"], kd=gains_dict["kd"], trim=gains_dict["trim"])

        cost_sum = 0.0
        for _ in range(cfg.auto.quick_steps):
            rs.phase += cfg.track.phase_speed * cfg.auto.shadow_dt

            cs.ctrl_accum += cfg.auto.shadow_dt
            while cs.ctrl_accum >= cfg.robot.dt_ctrl:
                cs.ctrl_accum -= cfg.robot.dt_ctrl

                e_front, _, _, _ = controller_error_front(cfg, rs.xw, rs.yw, rs.heading, rs.phase)

                cs.I_state += e_front * cfg.robot.dt_ctrl
                cs.I_state = clamp(cs.I_state, -5000.0, 5000.0)

                d_term = (e_front - cs.e_prev) / cfg.robot.dt_ctrl
                cs.e_prev = e_front

                cs.u_prev = cs.u
                cs.u = gains.kp * e_front + gains.ki * cs.I_state + gains.kd * d_term

                base_eff = base_pwm * speed_scale
                pwm_l_t = clamp(base_eff - cs.u + gains.trim, 0, 255)
                pwm_r_t = clamp(base_eff + cs.u - gains.trim, 0, 255)

                rs.pwm_l = ease_towards(rs.pwm_l, pwm_l_t, cfg.robot.dt_ctrl, tau=cfg.robot.actuator_lag_tau)
                rs.pwm_r = ease_towards(rs.pwm_r, pwm_r_t, cfg.robot.dt_ctrl, tau=cfg.robot.actuator_lag_tau)

            # kinematics
            v_l = (rs.pwm_l / 255.0) * cfg.robot.max_speed_px
            v_r = (rs.pwm_r / 255.0) * cfg.robot.max_speed_px
            v_l *= (1.0 + cfg.robot.steering_bias)
            v_r *= (1.0 - cfg.robot.steering_bias)

            v = 0.5 * (v_l + v_r)
            omega = (v_r - v_l) / cfg.robot.wheel_base

            rs.heading += omega * cfg.auto.shadow_dt
            rs.xw += v * math.cos(rs.heading) * cfg.auto.shadow_dt
            rs.yw += v * math.sin(rs.heading) * cfg.auto.shadow_dt
            rs.yw = clamp(rs.yw, 12, cfg.ui.height - 12)

            if cfg.robot.soft_pull_to_line:
                _, _, e_body = body_error(cfg, rs.xw, rs.yw, rs.heading, rs.phase)
                if abs(e_body) > cfg.robot.pull_threshold:
                    y_target = line_y_at(rs.xw, rs.phase, cfg.track)
                    rs.yw += (y_target - rs.yw) * clamp(cfg.robot.pull_strength * cfg.auto.shadow_dt, 0.0, 1.0)

            eF, eR, _ = body_error(cfg, rs.xw, rs.yw, rs.heading, rs.phase)
            e2 = 0.5 * (eF * eF + eR * eR)
            spin_r = omega / (abs(v) + 1e-3)
            du = cs.u - cs.u_prev

            cost_sum += (
                cfg.auto.w_e2 * e2
                + cfg.auto.w_spin2 * (omega * omega)
                + cfg.auto.w_spinr2 * (spin_r * spin_r)
                + cfg.auto.w_u2 * (cs.u * cs.u)
                + cfg.auto.w_du2 * (du * du)
            )

        return cost_sum / max(1, cfg.auto.quick_steps)

    def full_cycle_cost(self, snap: Dict, gains_dict: Dict, base_pwm: float, speed_scale: float) -> float:
        cfg = self.cfg
        rs, cs = _snapshot_to_state(cfg, snap)
        gains = PIDGains(kp=gains_dict["kp"], ki=gains_dict["ki"], kd=gains_dict["kd"], trim=gains_dict["trim"])

        s0 = rs.xw + rs.phase
        cost_sum = 0.0
        steps = 0

        while (rs.xw + rs.phase) - s0 < cfg.track.wavelen:
            rs.phase += cfg.track.phase_speed * cfg.auto.shadow_dt

            cs.ctrl_accum += cfg.auto.shadow_dt
            while cs.ctrl_accum >= cfg.robot.dt_ctrl:
                cs.ctrl_accum -= cfg.robot.dt_ctrl
                e_front, _, _, _ = controller_error_front(cfg, rs.xw, rs.yw, rs.heading, rs.phase)

                cs.I_state += e_front * cfg.robot.dt_ctrl
                cs.I_state = clamp(cs.I_state, -5000.0, 5000.0)

                d_term = (e_front - cs.e_prev) / cfg.robot.dt_ctrl
                cs.e_prev = e_front

                cs.u_prev = cs.u
                cs.u = gains.kp * e_front + gains.ki * cs.I_state + gains.kd * d_term

                base_eff = base_pwm * speed_scale
                pwm_l_t = clamp(base_eff - cs.u + gains.trim, 0, 255)
                pwm_r_t = clamp(base_eff + cs.u - gains.trim, 0, 255)

                rs.pwm_l = ease_towards(rs.pwm_l, pwm_l_t, cfg.robot.dt_ctrl, tau=cfg.robot.actuator_lag_tau)
                rs.pwm_r = ease_towards(rs.pwm_r, pwm_r_t, cfg.robot.dt_ctrl, tau=cfg.robot.actuator_lag_tau)

            v_l = (rs.pwm_l / 255.0) * cfg.robot.max_speed_px
            v_r = (rs.pwm_r / 255.0) * cfg.robot.max_speed_px
            v_l *= (1.0 + cfg.robot.steering_bias)
            v_r *= (1.0 - cfg.robot.steering_bias)

            v = 0.5 * (v_l + v_r)
            omega = (v_r - v_l) / cfg.robot.wheel_base

            rs.heading += omega * cfg.auto.shadow_dt
            rs.xw += v * math.cos(rs.heading) * cfg.auto.shadow_dt
            rs.yw += v * math.sin(rs.heading) * cfg.auto.shadow_dt
            rs.yw = clamp(rs.yw, 12, cfg.ui.height - 12)

            if cfg.robot.soft_pull_to_line:
                _, _, e_body = body_error(cfg, rs.xw, rs.yw, rs.heading, rs.phase)
                if abs(e_body) > cfg.robot.pull_threshold:
                    y_target = line_y_at(rs.xw, rs.phase, cfg.track)
                    rs.yw += (y_target - rs.yw) * clamp(cfg.robot.pull_strength * cfg.auto.shadow_dt, 0.0, 1.0)

            eF, eR, _ = body_error(cfg, rs.xw, rs.yw, rs.heading, rs.phase)
            e2 = 0.5 * (eF * eF + eR * eR)
            spin_r = omega / (abs(v) + 1e-3)
            du = cs.u - cs.u_prev

            cost_sum += (
                cfg.auto.w_e2 * e2
                + cfg.auto.w_spin2 * (omega * omega)
                + cfg.auto.w_spinr2 * (spin_r * spin_r)
                + cfg.auto.w_u2 * (cs.u * cs.u)
                + cfg.auto.w_du2 * (du * du)
            )

            steps += 1
            if steps > 3500:
                break

        return cost_sum / max(1, steps)