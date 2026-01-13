from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Colors:
    black: tuple[int, int, int] = (0, 0, 0)
    white: tuple[int, int, int] = (245, 245, 245)
    gray: tuple[int, int, int] = (140, 140, 140)
    dgray: tuple[int, int, int] = (60, 60, 60)


@dataclass(frozen=True)
class UIConfig:
    width: int = 1160
    height: int = 680
    hud_width: int = 470
    fps: int = 60

    @property
    def play_width(self) -> int:
        return self.width - self.hud_width


@dataclass(frozen=True)
class TrackConfig:
    center_y: int
    amp: float = 90.0
    wavelen: float = 980.0
    phase_speed: float = 50.0


@dataclass(frozen=True)
class RobotConfig:
    max_speed_px: float = 130.0
    wheel_base: float = 44.0
    robot_len: float = 34.0
    robot_wid: float = 22.0

    lookahead: float = 36.0

    sensor_noise_std: float = 4.0
    steering_bias: float = 0.05
    actuator_lag_tau: float = 0.18

    soft_pull_to_line: bool = True
    pull_threshold: float = 210.0
    pull_strength: float = 1.05

    # Controller update rate
    ctrl_hz: float = 18.0

    # World shift for long run stability
    world_shift_at: float = 1_000_000.0
    world_shift_by: float = 800_000.0

    @property
    def dt_ctrl(self) -> float:
        return 1.0 / self.ctrl_hz

    @property
    def body_front_offset(self) -> float:
        return self.robot_len * 0.50

    @property
    def body_rear_offset(self) -> float:
        return -self.robot_len * 0.50


@dataclass(frozen=True)
class CameraConfig:
    deadzone_left_ratio: float = 0.10
    deadzone_right_ratio: float = 0.90
    smooth_tau: float = 0.15
    safe_margin: int = 60


@dataclass(frozen=True)
class ChartConfig:
    seconds: float = 8.0
    cycle_hist_len: int = 70


@dataclass(frozen=True)
class PIDLimits:
    kp_min: float = 0.0
    kp_max: float = 5.0
    ki_min: float = 0.0
    ki_max: float = 0.2
    kd_min: float = 0.0
    kd_max: float = 10.0
    trim_min: float = -80.0
    trim_max: float = 80.0


@dataclass(frozen=True)
class AutoConfig:
    method_name: str = "Shadow Simulation Search"

    err_target: float = 20.0
    hold_stable_s: float = 4.5

    speed_scale: float = 0.78
    blend_tau: float = 0.60

    num_candidates: int = 14
    shadow_dt: float = 1.0 / 60.0
    quick_horizon_s: float = 1.10
    swap_margin: float = 0.96  # challenger must be <= 0.96*champ_cost to swap

    # Cost weights
    w_e2: float = 1.0
    w_spin2: float = 0.025
    w_spinr2: float = 0.010
    w_u2: float = 0.0010
    w_du2: float = 0.0030

    # Starting "bad" PID (to show improvement)
    bad_start: dict = None
    anchor: dict = None

    # Candidate radii
    rad_kp: float = 0.22
    rad_ki: float = 0.00025
    rad_kd: float = 0.55
    rad_tr: float = 7.0

    rad_grow: float = 1.15
    rad_shrink: float = 0.92
    rad_max: tuple[float, float, float, float] = (0.85, 0.0035, 2.4, 26.0)

    def __post_init__(self):
        # dataclass(frozen=True) needs object.__setattr__
        if self.bad_start is None:
            object.__setattr__(self, "bad_start", {"kp": 0.22, "ki": 0.0000, "kd": 0.15, "trim": 18.0})
        if self.anchor is None:
            object.__setattr__(self, "anchor", {"kp": 0.95, "ki": 0.00055, "kd": 1.35, "trim": 0.0})

    @property
    def quick_steps(self) -> int:
        return int(self.quick_horizon_s / self.shadow_dt)


@dataclass(frozen=True)
class AppConfig:
    ui: UIConfig
    colors: Colors
    track: TrackConfig
    robot: RobotConfig
    cam: CameraConfig
    charts: ChartConfig
    pid_limits: PIDLimits
    auto: AutoConfig

    @staticmethod
    def default() -> "AppConfig":
        ui = UIConfig()
        colors = Colors()
        track = TrackConfig(center_y=int(ui.height * 0.58))
        robot = RobotConfig()
        cam = CameraConfig()
        charts = ChartConfig()
        pid_limits = PIDLimits()
        auto = AutoConfig()
        return AppConfig(ui, colors, track, robot, cam, charts, pid_limits, auto)