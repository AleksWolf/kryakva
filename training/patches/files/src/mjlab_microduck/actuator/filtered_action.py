"""Joint position action with a first-order low-pass on the target (switch MICRODUCK_ACTION_FILTER_TAU, seconds).

The XL330 of the stock recipe (position-loop bandwidth ~2.4 Hz) smooths the 50 Hz policy output mechanically;
the HD-1910 (~8.5 Hz) tracks it literally. This term applies the smoothing in software:

    target <- target + alpha * (policy_target - target),   alpha = dt / (tau + dt)

Applied after scale/offset/clip, per env, reset to the default pose on episode reset. The same filter
must run in the robot runtime before the targets are sent to the servos (the ONNX holds only the network
and the observation normalizer).
"""

from __future__ import annotations

from dataclasses import dataclass

import torch
from mjlab.envs.mdp.actions.actions import JointPositionAction, JointPositionActionCfg


class FilteredJointPositionAction(JointPositionAction):
    cfg: "FilteredJointPositionActionCfg"

    def __init__(self, cfg: "FilteredJointPositionActionCfg", env) -> None:
        super().__init__(cfg=cfg, env=env)
        dt = float(env.step_dt)
        self._alpha = dt / (cfg.tau_s + dt)
        self._filt = torch.zeros_like(self._processed_actions)
        self._filt[:] = self._rest_target()
        print(f"[FilteredJointPositionAction] tau={cfg.tau_s * 1000:.0f} ms, alpha={self._alpha:.3f} at dt={dt * 1000:.0f} ms")

    def _rest_target(self) -> torch.Tensor:
        return self._entity.data.default_joint_pos[:, self._target_ids]

    def process_actions(self, actions: torch.Tensor) -> None:
        super().process_actions(actions)
        self._filt = self._filt + self._alpha * (self._processed_actions - self._filt)
        self._processed_actions = self._filt

    def reset(self, env_ids=None) -> None:
        super().reset(env_ids)
        ids = slice(None) if env_ids is None else env_ids
        self._filt[ids] = self._rest_target()[ids]


@dataclass(kw_only=True)
class FilteredJointPositionActionCfg(JointPositionActionCfg):
    tau_s: float = 0.06
    """Filter time constant, s (0.06 s ~ 2.6 Hz, the XL330's position-loop bandwidth)."""

    def build(self, env) -> FilteredJointPositionAction:
        return FilteredJointPositionAction(self, env)
