"""Protective fall (FallSafe) reward terms for the HD-1910 duck.

Runtime chain: walk -> (trunk tilt > theta_on) -> FallSafe -> (lying still) -> StandUp.
The policy starts at a recorded hand-over state (fall bank) and is paid for a soft landing and for lying still at
the end, or for staying on its feet. Impact costs are Pollen's VelStand functions (mdp.body_impact_cost,
mdp.servo_acc_spike_penalty); the end-state terms are here.
"""

import torch

from mjlab.envs import ManagerBasedRlEnv
from mjlab.managers.scene_entity_config import SceneEntityCfg


def _in_end_window(env: ManagerBasedRlEnv, window_s: float) -> torch.Tensor:
    t = env.episode_length_buf.float() * env.step_dt
    return t >= env.max_episode_length_s - window_s


def _tilt_z(env: ManagerBasedRlEnv, asset_cfg: SceneEntityCfg):
    asset = env.scene[asset_cfg.name]
    g = asset.data.projected_gravity_b
    tilt = torch.acos((-g[:, 2]).clamp(-1.0, 1.0))
    z = asset.data.root_link_pos_w[:, 2] - env.scene.terrain.env_origins[:, 2]
    return asset, tilt, z


def fs_end_upright(env: ManagerBasedRlEnv, window_s: float = 0.5, tilt_max: float = 0.175, z_min: float = 0.105,
                   fell_tilt: float = 0.785, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """1 per step in the last window_s of the episode while standing (tilt < tilt_max rad, trunk z > z_min m) and the
    trunk never tilted past fell_tilt (rad) in this episode."""
    _, tilt, z = _tilt_z(env, asset_cfg)
    mx = getattr(env, "_fs_max_tilt", None)
    if mx is None or mx.shape != tilt.shape:
        mx = torch.zeros_like(tilt)
    mx = torch.where(env.episode_length_buf <= 1, tilt, torch.maximum(mx, tilt))
    env._fs_max_tilt = mx
    ok = (tilt < tilt_max) & (z > z_min) & (mx < fell_tilt)
    return (ok & _in_end_window(env, window_s)).float()


def fs_end_rest(env: ManagerBasedRlEnv, window_s: float = 0.5, omega_std: float = 1.0, qd_std: float = 2.0,
                upright_tilt: float = 0.175, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Stillness score in the last window_s while down (not upright): exp(-|w_trunk|^2/omega_std^2) x
    exp(-mean qd^2 / qd_std^2)."""
    asset, tilt, _ = _tilt_z(env, asset_cfg)
    w = asset.data.root_link_ang_vel_b.norm(dim=-1)
    qd = (asset.data.joint_vel[:, asset_cfg.joint_ids] ** 2).mean(-1)
    score = torch.exp(-(w / omega_std) ** 2) * torch.exp(-qd / (qd_std * qd_std))
    down = tilt >= upright_tilt
    return torch.nan_to_num(score, nan=0.0) * (down & _in_end_window(env, window_s)).float()
