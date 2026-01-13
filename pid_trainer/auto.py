from __future__ import annotations

import random
from dataclasses import dataclass
from typing import Dict, List, Tuple

from .config import AppConfig
from .shadow import ShadowSimulator
from .utils import clamp


@dataclass
class AutoDecision:
    last_pick_tag: str = "none"
    last_swap: str = "NO"
    pred_champ: float | None = None
    pred_chall: float | None = None


class ShadowSimulationSearch:
    """
    Champion vs Challenger tuning:
    - propose candidates near current champion (and near current gains)
    - quick horizon cost to select challenger
    - full-cycle validation to accept swap
    """

    def __init__(self, cfg: AppConfig):
        self.cfg = cfg
        self.shadow = ShadowSimulator(cfg)

        self.champion: Dict[str, float] = cfg.auto.bad_start.copy()
        self.challenger: Dict[str, float] = cfg.auto.anchor.copy()
        self.target: Dict[str, float] = self.champion.copy()

        self.rad_kp = cfg.auto.rad_kp
        self.rad_ki = cfg.auto.rad_ki
        self.rad_kd = cfg.auto.rad_kd
        self.rad_tr = cfg.auto.rad_tr

    def reset(self) -> None:
        self.champion = self.cfg.auto.bad_start.copy()
        self.challenger = self.cfg.auto.anchor.copy()
        self.target = self.champion.copy()
        self.rad_kp = self.cfg.auto.rad_kp
        self.rad_ki = self.cfg.auto.rad_ki
        self.rad_kd = self.cfg.auto.rad_kd
        self.rad_tr = self.cfg.auto.rad_tr

    def _clamp_gains(self, g: Dict[str, float]) -> Dict[str, float]:
        lim = self.cfg.pid_limits
        return {
            "kp": clamp(g["kp"], lim.kp_min, lim.kp_max),
            "ki": clamp(g["ki"], lim.ki_min, lim.ki_max),
            "kd": clamp(g["kd"], lim.kd_min, lim.kd_max),
            "trim": clamp(g["trim"], lim.trim_min, lim.trim_max),
        }

    def build_candidates(self, current: Dict[str, float]) -> List[Dict]:
        cfg = self.cfg
        cands: List[Dict] = []
        cands.append({**current, "tag": "cur"})
        cands.append({**self.champion, "tag": "champ"})
        cands.append({**cfg.auto.anchor, "tag": "anch"})

        for i in range(cfg.auto.num_candidates - len(cands)):
            center = self.champion if (i % 2 == 0) else current
            cand = {
                "kp": center["kp"] + random.uniform(-self.rad_kp, self.rad_kp),
                "ki": center["ki"] + random.uniform(-self.rad_ki, self.rad_ki),
                "kd": center["kd"] + random.uniform(-self.rad_kd, self.rad_kd),
                "trim": center["trim"] + random.uniform(-self.rad_tr, self.rad_tr),
                "tag": "rnd",
            }
            cand = self._clamp_gains(cand)
            cand["tag"] = "rnd"
            cands.append(cand)

        return cands

    def decide(self, snap: Dict, current: Dict[str, float], base_pwm: float) -> Tuple[Dict[str, float], AutoDecision]:
        cfg = self.cfg
        decision = AutoDecision()

        # quick pick challenger
        best_q = float("inf")
        best_c = None
        for cand in self.build_candidates(current):
            cost = self.shadow.quick_cost(snap, cand, base_pwm, cfg.auto.speed_scale)
            if cost < best_q:
                best_q = cost
                best_c = cand

        assert best_c is not None
        self.challenger = {k: best_c[k] for k in ("kp", "ki", "kd", "trim")}
        decision.last_pick_tag = best_c.get("tag", "rnd")

        # full-cycle validation
        decision.pred_champ = self.shadow.full_cycle_cost(snap, self.champion, base_pwm, cfg.auto.speed_scale)
        decision.pred_chall = self.shadow.full_cycle_cost(snap, self.challenger, base_pwm, cfg.auto.speed_scale)

        do_swap = decision.pred_chall < decision.pred_champ * cfg.auto.swap_margin

        if do_swap:
            self.champion = self.challenger.copy()
            self.target = self.champion.copy()
            decision.last_swap = "YES"

            # shrink radius (exploit)
            self.rad_kp = min(self.rad_kp * cfg.auto.rad_shrink, cfg.auto.rad_max[0])
            self.rad_ki = min(self.rad_ki * cfg.auto.rad_shrink, cfg.auto.rad_max[1])
            self.rad_kd = min(self.rad_kd * cfg.auto.rad_shrink, cfg.auto.rad_max[2])
            self.rad_tr = min(self.rad_tr * cfg.auto.rad_shrink, cfg.auto.rad_max[3])
        else:
            decision.last_swap = "NO"

            # grow radius (explore)
            self.rad_kp = min(self.rad_kp * cfg.auto.rad_grow, cfg.auto.rad_max[0])
            self.rad_ki = min(self.rad_ki * cfg.auto.rad_grow, cfg.auto.rad_max[1])
            self.rad_kd = min(self.rad_kd * cfg.auto.rad_grow, cfg.auto.rad_max[2])
            self.rad_tr = min(self.rad_tr * cfg.auto.rad_grow, cfg.auto.rad_max[3])

        return self.target.copy(), decision