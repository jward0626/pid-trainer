from __future__ import annotations

import math
import sys
from collections import deque

import pygame

from .auto import ShadowSimulationSearch
from .config import AppConfig
from .sim import PIDGains, init_world, step_sim, body_error
from .track import line_y_at
from .utils import clamp, draw_text, draw_text_right, ease_towards


def draw_panel(surface: pygame.Surface, rect: tuple[int, int, int, int], dgray: tuple[int, int, int]) -> None:
    pygame.draw.rect(surface, (10, 10, 10), rect, 0)
    pygame.draw.rect(surface, dgray, rect, 1)


def draw_line_chart(
    surface: pygame.Surface,
    rect: tuple[int, int, int, int],
    series_gray: list[float],
    series_white: list[float],
    ymin: float,
    ymax: float,
    font_small: pygame.font.Font,
    colors,
    y_unit: str = "",
) -> None:
    x0, y0, w, h = rect
    draw_panel(surface, rect, colors.dgray)

    ymin = float(ymin)
    ymax = float(ymax)
    if ymin >= ymax:
        ymin, ymax = 0.0, 1.0

    def y_to_px(v: float) -> int:
        v = clamp(v, ymin, ymax)
        t = (v - ymin) / (ymax - ymin)
        return int(y0 + (1.0 - t) * (h - 1))

    pygame.draw.line(surface, colors.dgray, (x0 + 1, y0 + h - 2), (x0 + w - 2, y0 + h - 2), 1)

    if ymin < 0.0 < ymax:
        y_mid = y_to_px(0.0)
        pygame.draw.line(surface, colors.dgray, (x0 + 1, y_mid), (x0 + w - 2, y_mid), 1)

    draw_text(surface, font_small, f"{ymax:.0f}{y_unit}", x0 + 6, y0 + 4, colors.dgray, max_w=w - 12)
    draw_text(surface, font_small, f"{ymin:.0f}{y_unit}", x0 + 6, y0 + h - 18, colors.dgray, max_w=w - 12)

    def draw_series(data: list[float], color: tuple[int, int, int]) -> None:
        if len(data) < 2:
            return
        n = len(data)
        pts = []
        for i, v in enumerate(data):
            px = int(x0 + 1 + (i / (n - 1)) * (w - 2))
            py = y_to_px(v)
            pts.append((px, py))
        pygame.draw.lines(surface, color, False, pts, 1)

    draw_series(series_gray, colors.gray)
    draw_series(series_white, colors.white)


def draw_robot(screen: pygame.Surface, sx: float, sy: float, heading: float, robot_len: float, robot_wid: float, color: tuple[int, int, int]) -> None:
    c = math.cos(heading)
    s = math.sin(heading)
    hx = robot_len / 2
    hy = robot_wid / 2
    corners = [(-hx, -hy), (-hx, hy), (hx, hy), (hx, -hy)]
    poly = []
    for lx, ly in corners:
        wx = sx + lx * c - ly * s
        wy = sy + lx * s + ly * c
        poly.append((wx, wy))
    pygame.draw.polygon(screen, color, poly, 2)
    fx = sx + (hx + 6) * c
    fy = sy + (hx + 6) * s
    pygame.draw.circle(screen, color, (int(fx), int(fy)), 3)


class PIDTrainerApp:
    def __init__(self, cfg: AppConfig | None = None):
        self.cfg = cfg or AppConfig.default()

        self.params = ["B", "P", "I", "D", "T"]
        self.selected = 0

        self.paused = False
        self.auto_state = "OFF"  # OFF / LEARN / HOLD

        self.base = 120.0

        # gains
        bs = self.cfg.auto.bad_start
        self.kp0, self.ki0, self.kd0, self.trim0 = bs["kp"], bs["ki"], bs["kd"], bs["trim"]

        # world/controller
        self.rs, self.cs = init_world(self.cfg, self.base)

        # camera
        self.cam_x = 0.0

        # charts
        self.chart_points = int(self.cfg.charts.seconds * self.cfg.ui.fps)
        self.hist_body = deque(maxlen=self.chart_points)
        self.hist_front_meas = deque(maxlen=self.chart_points)

        # avg error
        self.avg_abs_e = 0.0
        self.stable_s = 0.0

        # cycle tracking
        self.cycle_s0 = self.rs.xw + self.rs.phase
        self.cycle_progress = 0.0
        self.cycle_err_sum = 0.0
        self.cycle_err_count = 0

        # improvement chart
        self.cycle_err_hist = deque(maxlen=self.cfg.charts.cycle_hist_len)
        self.best_err_hist = deque(maxlen=self.cfg.charts.cycle_hist_len)
        self.swap_hist = deque(maxlen=self.cfg.charts.cycle_hist_len)
        self.best_cycle_err = float("inf")
        self.swap_count = 0
        self.cycle_count = 0

        # auto search
        self.search = ShadowSimulationSearch(self.cfg)
        self.target = self.search.champion.copy()
        self.last_pick = "none"
        self.last_swap = "NO"
        self.pred_champ = None
        self.pred_chall = None

        # display smoothing
        self.disp = {"kp": self.kp0, "ki": self.ki0, "kd": self.kd0, "tr": self.trim0}

    def reset_all(self) -> None:
        self.__init__(self.cfg)

    def apply_delta_manual(self, key: str, dv: float) -> None:
        lim = self.cfg.pid_limits
        if key == "B":
            self.base = clamp(self.base + dv, 0, 255)
        elif key == "P":
            self.kp0 = clamp(self.kp0 + dv, lim.kp_min, lim.kp_max)
        elif key == "I":
            self.ki0 = clamp(self.ki0 + dv, lim.ki_min, lim.ki_max)
        elif key == "D":
            self.kd0 = clamp(self.kd0 + dv, lim.kd_min, lim.kd_max)
        elif key == "T":
            self.trim0 = clamp(self.trim0 + dv, lim.trim_min, lim.trim_max)

        # keep target aligned when manual
        self.target = {"kp": self.kp0, "ki": self.ki0, "kd": self.kd0, "trim": self.trim0}

    def run(self) -> None:
        cfg = self.cfg
        pygame.init()
        pygame.display.set_caption("PID Trainer")
        screen = pygame.display.set_mode((cfg.ui.width, cfg.ui.height))
        clock = pygame.time.Clock()
        pygame.key.set_repeat(200, 35)

        font_title = pygame.font.SysFont("consolas", 24)
        font = pygame.font.SysFont("consolas", 17)
        font_small = pygame.font.SysFont("consolas", 14)

        running = True
        while running:
            dt = clock.tick(cfg.ui.fps) / 1000.0

            # ---------------- Events ----------------
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    if event.key == pygame.K_SPACE:
                        self.paused = not self.paused
                    if event.key == pygame.K_r:
                        self.reset_all()
                    if event.key == pygame.K_a:
                        if self.auto_state == "OFF":
                            self.auto_state = "LEARN"
                            self.stable_s = 0.0
                            self.cycle_s0 = self.rs.xw + self.rs.phase
                            self.cycle_err_sum = 0.0
                            self.cycle_err_count = 0
                            self.last_swap = "NO"
                            self.last_pick = "none"
                            # reset tuner to start from current gains
                            self.search.champion = {"kp": self.kp0, "ki": self.ki0, "kd": self.kd0, "trim": self.trim0}
                            self.search.target = self.search.champion.copy()
                            self.target = self.search.target.copy()
                        else:
                            self.auto_state = "OFF"
                            self.target = {"kp": self.kp0, "ki": self.ki0, "kd": self.kd0, "trim": self.trim0}

                    # manual only when auto OFF
                    if self.auto_state == "OFF":
                        if event.key == pygame.K_UP:
                            self.selected = (self.selected - 1) % len(self.params)
                        elif event.key == pygame.K_DOWN:
                            self.selected = (self.selected + 1) % len(self.params)
                        elif event.key in (pygame.K_LEFT, pygame.K_RIGHT):
                            key = self.params[self.selected]
                            if key == "B":
                                step = 2.0
                            elif key == "P":
                                step = 0.02
                            elif key == "I":
                                step = 0.0001
                            elif key == "D":
                                step = 0.05
                            else:
                                step = 0.5
                            self.apply_delta_manual(key, -step if event.key == pygame.K_LEFT else step)

            # ------------ Auto blend towards target ------------
            if self.auto_state in ("LEARN", "HOLD"):
                self.kp0 = ease_towards(self.kp0, self.target["kp"], dt, tau=cfg.auto.blend_tau)
                self.ki0 = ease_towards(self.ki0, self.target["ki"], dt, tau=cfg.auto.blend_tau)
                self.kd0 = ease_towards(self.kd0, self.target["kd"], dt, tau=cfg.auto.blend_tau)
                self.trim0 = ease_towards(self.trim0, self.target["trim"], dt, tau=cfg.auto.blend_tau)

            gains = PIDGains(self.kp0, self.ki0, self.kd0, self.trim0)

            # ---------------- Simulation step ----------------
            meas = step_sim(
                cfg,
                self.rs,
                self.cs,
                gains,
                base_pwm=self.base,
                dt=dt,
                paused=self.paused,
                speed_scale=(cfg.auto.speed_scale if self.auto_state in ("LEARN", "HOLD") else 1.0),
                measure_noise=True,
            )

            e_body = float(meas["e_body"])
            e_front_meas = float(meas["e_front_meas"])

            self.hist_body.append(e_body)
            self.hist_front_meas.append(e_front_meas)
            self.avg_abs_e = ease_towards(self.avg_abs_e, abs(e_body), dt, tau=2.2)

            # cycle error accumulation
            self.cycle_err_sum += abs(e_body)
            self.cycle_err_count += 1

            # ---------------- Cycle progress + auto state ----------------
            s = self.rs.xw + self.rs.phase
            self.cycle_progress = clamp((s - self.cycle_s0) / cfg.track.wavelen, 0.0, 1.0)

            if self.auto_state == "LEARN":
                if self.avg_abs_e <= cfg.auto.err_target:
                    self.stable_s += dt
                else:
                    self.stable_s = max(0.0, self.stable_s - 0.35 * dt)
                if self.stable_s >= cfg.auto.hold_stable_s:
                    self.auto_state = "HOLD"

            if self.auto_state == "HOLD":
                if self.avg_abs_e > (cfg.auto.err_target + 8.0):
                    self.auto_state = "LEARN"
                    self.stable_s = 0.0

            # ---------------- Cycle end: decide swap ----------------
            if (self.auto_state in ("LEARN", "HOLD")) and (not self.paused) and (self.cycle_progress >= 1.0 - 1e-6):
                cycle_avg_err = (self.cycle_err_sum / max(1, self.cycle_err_count))
                self.cycle_count += 1

                if cycle_avg_err < self.best_cycle_err:
                    self.best_cycle_err = cycle_avg_err

                self.cycle_err_hist.append(cycle_avg_err)
                self.best_err_hist.append(self.best_cycle_err)

                snap = {
                    "xw": self.rs.xw,
                    "yw": self.rs.yw,
                    "heading": self.rs.heading,
                    "phase": self.rs.phase,
                    "I_state": self.cs.I_state,
                    "e_prev": self.cs.e_prev,
                    "ctrl_accum": self.cs.ctrl_accum,
                    "pwm_l": self.rs.pwm_l,
                    "pwm_r": self.rs.pwm_r,
                    "u_prev": self.cs.u_prev,
                    "u": self.cs.u,
                }

                current = {"kp": self.kp0, "ki": self.ki0, "kd": self.kd0, "trim": self.trim0}
                self.target, decision = self.search.decide(snap, current, self.base)
                self.last_pick = decision.last_pick_tag
                self.last_swap = decision.last_swap
                self.pred_champ = decision.pred_champ
                self.pred_chall = decision.pred_chall

                did_swap = (self.last_swap == "YES")
                if did_swap:
                    self.swap_count += 1
                    self.swap_hist.append(True)
                    if self.auto_state == "HOLD":
                        self.auto_state = "LEARN"
                        self.stable_s = 0.0
                else:
                    self.swap_hist.append(False)

                # reset cycle
                self.cycle_s0 = self.rs.xw + self.rs.phase
                self.cycle_err_sum = 0.0
                self.cycle_err_count = 0

            while len(self.swap_hist) < len(self.cycle_err_hist):
                self.swap_hist.append(False)

            # ---------------- Camera follow ----------------
            play_w = cfg.ui.play_width
            dead_left = play_w * cfg.cam.deadzone_left_ratio
            dead_right = play_w * cfg.cam.deadzone_right_ratio

            sx_robot_now = self.rs.xw - self.cam_x
            target_cam_x = self.cam_x
            if sx_robot_now < dead_left:
                target_cam_x = self.rs.xw - dead_left
            elif sx_robot_now > dead_right:
                target_cam_x = self.rs.xw - dead_right

            self.cam_x = ease_towards(self.cam_x, target_cam_x, dt, tau=cfg.cam.smooth_tau)
            min_cam = self.rs.xw - (play_w - cfg.cam.safe_margin)
            max_cam = self.rs.xw - cfg.cam.safe_margin
            self.cam_x = clamp(self.cam_x, min_cam, max_cam)
            if self.cam_x < 0:
                self.cam_x = 0.0

            # smooth HUD values
            self.disp["kp"] = ease_towards(self.disp["kp"], self.kp0, dt, tau=0.18)
            self.disp["ki"] = ease_towards(self.disp["ki"], self.ki0, dt, tau=0.18)
            self.disp["kd"] = ease_towards(self.disp["kd"], self.kd0, dt, tau=0.18)
            self.disp["tr"] = ease_towards(self.disp["tr"], self.trim0, dt, tau=0.18)

            # ---------------- Render ----------------
            c = cfg.colors
            screen.fill(c.black)
            pygame.draw.rect(screen, c.dgray, (0, 0, play_w, cfg.ui.height), 2)

            # track
            pts = []
            for sx in range(0, play_w + 1, 8):
                x_world = sx + self.cam_x
                pts.append((sx, line_y_at(x_world, self.rs.phase, cfg.track)))
            pygame.draw.lines(screen, c.white, False, pts, 2)
            offset = 22
            pygame.draw.lines(screen, c.dgray, False, [(px, py - offset) for (px, py) in pts], 1)
            pygame.draw.lines(screen, c.dgray, False, [(px, py + offset) for (px, py) in pts], 1)

            # robot
            sx_robot = self.rs.xw - self.cam_x
            sy_robot = self.rs.yw
            draw_robot(screen, sx_robot, sy_robot, self.rs.heading, cfg.robot.robot_len, cfg.robot.robot_wid, c.white)

            # lookahead marker
            la_sx = float(meas["la_xw"]) - self.cam_x
            la_sy = float(meas["la_yw"])
            y_line = float(meas["y_line"])
            pygame.draw.circle(screen, c.gray, (int(la_sx), int(la_sy)), 3, 1)
            pygame.draw.line(screen, c.gray, (int(la_sx), int(la_sy)), (int(la_sx), int(y_line)), 1)

            # HUD
            hud_x = play_w
            margin = 16
            inner_w = cfg.ui.hud_width - 2 * margin

            pygame.draw.rect(screen, (10, 10, 10), (hud_x, 0, cfg.ui.hud_width, cfg.ui.height))
            pygame.draw.line(screen, c.dgray, (hud_x, 0), (hud_x, cfg.ui.height), 2)

            # fixed layout (no overlap)
            top_h = 112
            params_h = 170
            bottom_h = 140
            gap = 10
            top_y = 14
            bottom_margin = 14

            r_top = (hud_x + margin, top_y, inner_w, top_h)
            r_params = (hud_x + margin, r_top[1] + top_h + gap, inner_w, params_h)

            chart_y = r_params[1] + params_h + gap
            chart_h = cfg.ui.height - (chart_y + gap + bottom_h + bottom_margin)
            chart_h = max(170, chart_h)

            r_chart = (hud_x + margin, chart_y, inner_w, chart_h)
            r_bottom = (hud_x + margin, chart_y + chart_h + gap, inner_w, bottom_h)

            # TOP
            draw_panel(screen, r_top, c.dgray)
            x0, y0, w0, _ = r_top
            draw_text(screen, font_title, "PID TRAINER", x0 + 10, y0 + 8, c.white, max_w=w0 - 20)
            draw_text_right(screen, font_small, "by Jacky Li from HKUST", x0 + w0 - 10, y0 + 12, c.gray, max_w=w0 - 20)

            draw_text(screen, font, f"Pause: {'YES' if self.paused else 'NO'}", x0 + 10, y0 + 40, c.gray, max_w=w0 - 20)
            draw_text(screen, font, f"Avg error: {self.avg_abs_e:6.2f} px", x0 + 10, y0 + 62, c.gray, max_w=w0 - 20)
            draw_text(screen, font_small, f"AUTO: {self.auto_state}  |  {cfg.auto.method_name}", x0 + 10, y0 + 86, c.gray, max_w=w0 - 20)

            # PARAMS
            draw_panel(screen, r_params, c.dgray)
            x1, y1, w1, _ = r_params

            if self.auto_state == "OFF":
                sel_key = self.params[self.selected]
                draw_text(screen, font_small, "Keys: UP DOWN select B P I D T, LEFT RIGHT change", x1 + 10, y1 + 10, c.gray, max_w=w1 - 20)
                draw_text(screen, font_small, f"Select: {sel_key}  A auto  R reset  SPACE pause", x1 + 10, y1 + 28, c.gray, max_w=w1 - 20)
            else:
                draw_text(screen, font_small, f"Cycle progress: {self.cycle_progress*100:5.1f}%", x1 + 10, y1 + 10, c.gray, max_w=w1 - 20)
                draw_text(screen, font_small, f"Pick={self.last_pick} Swap={self.last_swap} Swaps={self.swap_count}", x1 + 10, y1 + 28, c.gray, max_w=w1 - 20)

            def fmt_line(key: str, label: str, value_str: str) -> str:
                mark = ">" if (self.auto_state == "OFF" and self.params[self.selected] == key) else " "
                return f"{mark} {label}: {value_str}"

            draw_text(screen, font, fmt_line("B", "B Base", f"{self.base:8.2f}"), x1 + 10, y1 + 52, c.white, max_w=w1 - 20)
            draw_text(screen, font, fmt_line("P", "P Kp", f"{self.disp['kp']:8.3f}"), x1 + 10, y1 + 78, c.white, max_w=w1 - 20)
            draw_text(screen, font, fmt_line("I", "I Ki", f"{self.disp['ki']:8.5f}"), x1 + 10, y1 + 104, c.white, max_w=w1 - 20)
            draw_text(screen, font, fmt_line("D", "D Kd", f"{self.disp['kd']:8.3f}"), x1 + 10, y1 + 130, c.white, max_w=w1 - 20)
            draw_text(screen, font, fmt_line("T", "T Trim", f"{self.disp['tr']:8.2f}"), x1 + 10, y1 + 156, c.white, max_w=w1 - 20)

            # ERROR CHART
            draw_panel(screen, r_chart, c.dgray)
            x2, y2, w2, h2 = r_chart
            draw_text(screen, font, "Error chart", x2 + 10, y2 + 10, c.gray, max_w=w2 - 20)
            chart_rect = (x2 + 10, y2 + 34, w2 - 20, h2 - 44)
            recent = list(self.hist_body)
            amp = max(55.0, (max((abs(v) for v in recent), default=0.0) * 1.25))
            draw_line_chart(
                screen,
                chart_rect,
                series_gray=list(self.hist_body),
                series_white=list(self.hist_front_meas),
                ymin=-amp,
                ymax=amp,
                font_small=font_small,
                colors=c,
                y_unit="px",
            )

            # IMPROVEMENT CHART
            draw_panel(screen, r_bottom, c.dgray)
            xb, yb, wb, hb = r_bottom
            draw_text(screen, font_small, "Improvement chart", xb + 10, yb + 8, c.gray, max_w=wb - 20)
            imp_rect = (xb + 10, yb + 28, wb - 20, hb - 38)

            if len(self.cycle_err_hist) >= 2:
                mx = max(self.cycle_err_hist)
                ymin = 0.0
                ymax = max(30.0, mx * 1.15)
                draw_line_chart(
                    screen,
                    imp_rect,
                    series_gray=list(self.best_err_hist),
                    series_white=list(self.cycle_err_hist),
                    ymin=ymin,
                    ymax=ymax,
                    font_small=font_small,
                    colors=c,
                    y_unit="px",
                )
                last = self.cycle_err_hist[-1]
                best = self.best_err_hist[-1]
                x0i, y0i, w0i, _ = imp_rect
                draw_text(screen, font_small, f"last={last:.1f}px  best={best:.1f}px", x0i + 8, y0i + 6, c.white, max_w=w0i - 16)

                x0i, y0i, w0i, h0i = imp_rect
                n = len(self.cycle_err_hist)
                for i, did_swap in enumerate(self.swap_hist):
                    if not did_swap:
                        continue
                    px = int(x0i + 1 + (i / (n - 1)) * (w0i - 2))
                    pygame.draw.line(screen, c.dgray, (px, y0i + h0i - 2), (px, y0i + h0i - 10), 1)
            else:
                draw_panel(screen, imp_rect, c.dgray)
                draw_text(screen, font_small, "Run AUTO for 2+ cycles.", imp_rect[0] + 10, imp_rect[1] + 10, c.gray, max_w=imp_rect[2] - 20)

            pygame.display.flip()

        pygame.quit()
        sys.exit()