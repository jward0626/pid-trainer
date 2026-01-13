from __future__ import annotations

import math
from typing import Optional

import pygame


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def ease_towards(current: float, target: float, dt: float, tau: float = 0.22) -> float:
    """First-order lag to make transitions smooth and stable."""
    if tau <= 1e-6:
        return target
    alpha = 1.0 - math.exp(-dt / tau)
    return current + (target - current) * alpha


def fit_text(font: pygame.font.Font, text: str, max_w: int) -> str:
    """Ellipsize text to fit max width in pixels."""
    if max_w <= 10:
        return ""
    if font.size(text)[0] <= max_w:
        return text
    ell = "..."
    lo, hi = 0, len(text)
    best = ell
    while lo <= hi:
        mid = (lo + hi) // 2
        cand = text[:mid] + ell
        if font.size(cand)[0] <= max_w:
            best = cand
            lo = mid + 1
        else:
            hi = mid - 1
    return best


def draw_text(
    surface: pygame.Surface,
    font: pygame.font.Font,
    text: str,
    x: int,
    y: int,
    color: tuple[int, int, int],
    max_w: Optional[int] = None,
) -> tuple[int, int]:
    if max_w is not None:
        text = fit_text(font, text, max_w)
    img = font.render(text, True, color)
    surface.blit(img, (x, y))
    return img.get_width(), img.get_height()


def draw_text_right(
    surface: pygame.Surface,
    font: pygame.font.Font,
    text: str,
    right_x: int,
    y: int,
    color: tuple[int, int, int],
    max_w: Optional[int] = None,
) -> tuple[int, int]:
    if max_w is not None:
        text = fit_text(font, text, max_w)
    img = font.render(text, True, color)
    surface.blit(img, (right_x - img.get_width(), y))
    return img.get_width(), img.get_height()