"""Velocity command variant for the HD-1910 duck, used only through an env switch.

TurnInPlaceWideCommand: the recipe's turn-in-place bucket (VelocityCommandCommandOnly: lin = 0,
|wz| ~ U(0.4·max, max), max = ranges.ang_vel_z) with its own upper bound. The walking/turning
combinations keep the recipe's ranges.ang_vel_z (±1.0 rad/s).
"""

from __future__ import annotations

from dataclasses import dataclass

import torch

from mjlab_microduck.tasks.mdp import VelocityCommandCommandOnly, VelocityCommandCommandOnlyCfg


class TurnInPlaceWideCommand(VelocityCommandCommandOnly):
    def _resample_command(self, env_ids: torch.Tensor) -> None:
        super()._resample_command(env_ids)
        wide = float(getattr(self.cfg, "turn_in_place_max", 0.0))
        lo, hi = self.cfg.ranges.ang_vel_z
        maxr = max(abs(lo), abs(hi))
        if wide <= maxr or len(env_ids) == 0:
            return
        c = self.vel_command_b[env_ids]
        # the recipe's turn-in-place envs: zero linear command, |wz| >= 0.4·max, not standing
        turn = (c[:, 0] == 0) & (c[:, 1] == 0) & (c[:, 2].abs() >= 0.4 * maxr - 1e-6) & ~self.is_standing_env[env_ids]
        ids = env_ids[turn]
        if len(ids) == 0:
            return
        sign = torch.sign(self.vel_command_b[ids, 2])
        mag = torch.empty(len(ids), device=self.device).uniform_(0.4 * maxr, wide)
        self.vel_command_b[ids, 2] = sign * mag
        self.vel_command_w[ids] = self.vel_command_b[ids]


@dataclass(kw_only=True)
class TurnInPlaceWideCommandCfg(VelocityCommandCommandOnlyCfg):
    turn_in_place_max: float = 0.0   # upper |wz| of the turn-in-place bucket, rad/s (0 = recipe's ranges.ang_vel_z)

    def build(self, env) -> TurnInPlaceWideCommand:
        return TurnInPlaceWideCommand(self, env)

