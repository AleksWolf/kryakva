"""Reset events for the HD-1910 duck.

reset_from_fall_bank: a share of StandUp episodes start from a recorded fall state (the robot state at the moment
the runtime switches walk -> StandUp at trunk tilt > 35 deg after a push) instead of a keyframe spawn. Each sample
is rotated by a random yaw.
"""

from __future__ import annotations

import math

import numpy as np
import torch
from mjlab.envs import ManagerBasedRlEnv


def reset_from_fall_bank(env: ManagerBasedRlEnv, env_ids: torch.Tensor, path: str, prob: float) -> None:
    if env_ids is None or len(env_ids) == 0:
        return
    bank = getattr(env, "_fall_bank", None)
    robot = env.scene["robot"]
    if bank is None:
        b = np.load(path)
        names = list(robot.joint_names)
        src = [str(x) for x in b["joint_names"]]
        cols = [(names.index(nm), j) for j, nm in enumerate(src) if nm in names]
        dev = env.device
        bank = {k: torch.tensor(b[k], dtype=torch.float32, device=dev)
                for k in ("root_pos", "root_quat", "root_lin_vel", "root_ang_vel", "joint_pos", "joint_vel")}
        bank["dst"] = torch.tensor([c[0] for c in cols], device=dev)
        bank["src"] = torch.tensor([c[1] for c in cols], device=dev)
        env._fall_bank = bank
        print(f"[events_hd1910] fall bank {path}: {len(b['joint_pos'])} states, {len(cols)} joints mapped, prob {prob}")
    sel = env_ids[torch.rand(len(env_ids), device=env.device) < prob]
    if len(sel) == 0:
        return
    k = torch.randint(0, bank["joint_pos"].shape[0], (len(sel),), device=env.device)
    yaw = torch.rand(len(sel), device=env.device) * 2 * math.pi
    c, s = torch.cos(yaw), torch.sin(yaw)
    qz = torch.stack([torch.cos(yaw / 2), torch.zeros_like(yaw), torch.zeros_like(yaw), torch.sin(yaw / 2)], dim=1)
    q = bank["root_quat"][k]                                             # (w, x, y, z)
    w1, x1, y1, z1 = qz.unbind(1)
    w2, x2, y2, z2 = q.unbind(1)
    quat = torch.stack([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2], dim=1)

    def rot(v):
        return torch.stack([c * v[:, 0] - s * v[:, 1], s * v[:, 0] + c * v[:, 1], v[:, 2]], dim=1)

    pos = bank["root_pos"][k].clone()
    pos[:, :2] = 0.0
    pos = pos + env.scene.env_origins[sel]
    root = torch.cat([pos, quat, rot(bank["root_lin_vel"][k]), rot(bank["root_ang_vel"][k])], dim=1)
    robot.write_root_state_to_sim(root, env_ids=sel)
    jp = robot.data.default_joint_pos[sel].clone()
    jv = torch.zeros_like(jp)
    jp[:, bank["dst"]] = bank["joint_pos"][k][:, bank["src"]]
    jv[:, bank["dst"]] = bank["joint_vel"][k][:, bank["src"]]
    robot.write_joint_state_to_sim(jp, jv, env_ids=sel)
