from __future__ import annotations

import math

from .config import TrackConfig


def line_y_at(x_world: float, phase: float, track: TrackConfig) -> float:
    return track.center_y + track.amp * math.sin((x_world + phase) / track.wavelen * 2.0 * math.pi)


def line_dydx_at(x_world: float, phase: float, track: TrackConfig) -> float:
    k = (2.0 * math.pi) / track.wavelen
    return track.amp * k * math.cos(k * (x_world + phase))


def track_heading_at(x_world: float, phase: float, track: TrackConfig) -> float:
    return math.atan2(line_dydx_at(x_world, phase, track), 1.0)