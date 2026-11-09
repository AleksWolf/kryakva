"""Reward variants for the HD-1910 recipes.

track_angular_velocity_dual_std: the stock term exp(-err^2/std^2) prices yaw error the same whether a
turn is commanded or not, so the small yaw wobble of a fast straight walk is taxed at the same rate as a
failed turn. Here the std is wider when no turn is commanded.
"""

from __future__ import annotations

import math

import torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.managers.scene_entity_config import SceneEntityCfg

_ROBOT = SceneEntityCfg("robot")


def track_angular_velocity_dual_std(
    env: ManagerBasedRlEnv,
    std: float,
    std_zero_cmd: float,
    command_name: str,
    zero_cmd_threshold: float = 0.05,
    asset_cfg: SceneEntityCfg = _ROBOT,
) -> torch.Tensor:
    asset = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    assert command is not None, f"Command '{command_name}' not found."
    actual = asset.data.root_link_ang_vel_b
    z_error = torch.square(command[:, 2] - actual[:, 2])
    xy_error = torch.sum(torch.square(actual[:, :2]), dim=1)
    err = z_error + xy_error
    turning = torch.abs(command[:, 2]) > zero_cmd_threshold
    s2 = torch.where(turning, torch.full_like(err, std**2), torch.full_like(err, std_zero_cmd**2))
    return torch.exp(-err / s2)


def head_ground_at_rest(
    env: ManagerBasedRlEnv,
    sensor_name: str = "head_ground_contact",
    command_name: str = "twist",
) -> torch.Tensor:
    """1.0 while the head touches the floor AND the SitStand posture transition is over, else 0.

    The contact counts only at rest: the slewed target blend (SitStandCommand.alpha) has reached the
    commanded flag. Any contact counts once per step (not per contact point).
    """
    found = env.scene[sensor_name].data.found
    touching = (found.reshape(found.shape[0], -1) > 0).any(dim=1)
    term = env.command_manager.get_term(command_name)
    at_rest = torch.abs(term.alpha - term.command[:, 0]) < 1e-3
    return (touching & at_rest).float()


def head_ground_touch(
    env: ManagerBasedRlEnv,
    sensor_name: str = "head_ground_contact",
) -> torch.Tensor:
    """1.0 while the head (jaw_soft) touches the floor, in any phase; once per step, not per contact point."""
    found = env.scene[sensor_name].data.found
    return (found.reshape(found.shape[0], -1) > 0).any(dim=1).float()


def knee_sit_progress(
    env: ManagerBasedRlEnv,
    command_name: str = "twist",
    sit_knee: float = 1.30,
    asset_cfg: SceneEntityCfg = _ROBOT,
) -> torch.Tensor:
    """Pay each new gain of knee flexion toward the sit pose under a SIT command (frontier, per step, / dt).

    p = mean over both knees of clamp((|q_knee| - |q_knee_HOME|) / (sit_knee - |q_knee_HOME|), 0, 1). Under the sit flag
    the reward is max(0, p - frontier) / dt and the frontier moves up to p; under the stand flag and on episode start the
    frontier follows p. A full sit pays 1 (x weight); rocking or holding pays nothing; trunk lean pays nothing.
    """
    asset = env.scene[asset_cfg.name]
    ids = [asset.joint_names.index(n) for n in ("left_knee", "right_knee")]
    q = asset.data.joint_pos[:, ids].abs(); h = asset.data.default_joint_pos[:, ids].abs()
    p = torch.clamp((q - h) / (sit_knee - h), 0.0, 1.0).mean(dim=1)
    # Capped by the command's slewed target blend (alpha: 0 stand -> 1 sit over the recipe's 2 s ramp): being ahead of the
    # ramp pays nothing.
    term = env.command_manager.get_term(command_name)
    alpha = getattr(term, "alpha", None)
    if alpha is not None:
        p = torch.minimum(p, alpha.reshape(-1))
    flag = env.command_manager.get_command(command_name)[:, 0] > 0.5
    front = getattr(env, "_knee_sit_front", None)
    if front is None or front.shape[0] != p.shape[0]:
        front = p.clone()
    fresh = (env.episode_length_buf <= 1) | ~flag
    gain = torch.where(fresh, torch.zeros_like(p), torch.clamp(p - front, min=0.0))
    env._knee_sit_front = torch.where(fresh, p, torch.maximum(front, p))
    return gain / env.step_dt


def tilt_over_limit(
    env: ManagerBasedRlEnv,
    limit_deg: float = 20.0,
    asset_cfg: SceneEntityCfg = _ROBOT,
) -> torch.Tensor:
    """relu(trunk tilt - limit)^2, rad^2: zero up to <limit>, any phase."""
    asset = env.scene[asset_cfg.name]
    g = asset.data.projected_gravity_b
    tilt = torch.acos(torch.clamp(-g[:, 2], -1.0, 1.0))
    over = torch.clamp(tilt - math.radians(limit_deg), min=0.0)
    return torch.nan_to_num(over * over, nan=0.0)


def upright_sharp_standing(
    env: ManagerBasedRlEnv,
    std_deg: float = 3.0,
    command_name: str = "twist",
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """exp(-(tilt / std)^2) of the true trunk tilt, only while standing at rest (STAND commanded, blend finished).

    The recipe's upright terms are cos(tilt), flat near vertical; this sharp true-tilt peak makes the policy hold
    the trunk by leg kinematics rather than by a possibly misaligned IMU. The critic sees the true tilt.
    """
    import math
    g = env.scene[asset_cfg.name].data.projected_gravity_b
    tilt = torch.acos((-g[:, 2]).clamp(-1.0, 1.0))
    term = env.command_manager.get_term(command_name)
    standing_rest = (term.command[:, 0] < 0.5) & (term.alpha < 1e-3)
    return torch.exp(-(tilt / math.radians(std_deg)) ** 2) * standing_rest.float()


def posture_descent_bootstrap(
    env: ManagerBasedRlEnv,
    command_name: str = "twist",
    min_height: float = 0.07,
    max_vz: float = 0.05,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Downward-vz reward, only while SIT is commanded and the trunk is above ``min_height``: the mirror of the
    recipe's ``posture_rise_bootstrap`` (tasks/mdp.py). Capped at ``max_vz`` (the recipe's MAX_DESCENT_SPEED).
    """
    asset = env.scene[asset_cfg.name]
    sit = env.command_manager.get_command(command_name)[:, 0]
    z = torch.nan_to_num(asset.data.root_link_pos_w[:, 2] - env.scene.terrain.env_origins[:, 2], nan=0.0)
    vz = torch.nan_to_num(asset.data.root_link_lin_vel_w[:, 2], nan=0.0)
    return torch.clamp(-vz, min=0.0, max=max_vz) * (z > min_height).float() * sit


def servo_torque_over_limit(
    env: ManagerBasedRlEnv,
    frac: float = 0.6,
    stall_nm: float = 1.37,
    skip_s: float = 0.0,
    asset_cfg: SceneEntityCfg = _ROBOT,
) -> torch.Tensor:
    """Sum over servos of relu(|tau| / stall - frac)^2: zero while every servo stays below frac of its stall torque.

    stall_nm = HD-1910 stall at 7.4 V (kt 0.692 · 7.4 / R 3.75). Actuator torque = qfrc_actuator on the
    non-passive joints.
    """
    asset = env.scene[asset_cfg.name]
    names = list(asset.joint_names)
    ids = [i for i, n in enumerate(names) if not n.startswith("passive_")]
    tau = asset.data.qfrc_actuator[:, ids]
    over = torch.clamp(tau.abs() / stall_nm - frac, min=0.0)
    pen = torch.sum(over * over, dim=1)
    if skip_s > 0.0:
        # skip the first skip_s of each episode: the spawn drop (robot appears above the floor) gives an impact the
        # real robot never sees.
        pen = pen * (env.episode_length_buf.float() * env.step_dt >= skip_s).float()
    return pen


def servo_torque_rate(env: ManagerBasedRlEnv, stall_nm: float = 1.37, joints: str = "", asset_cfg: SceneEntityCfg = _ROBOT) -> torch.Tensor:
    """Sum over the physics steps of this policy step and over the servos of (dtau / stall)^2: torque jerk in the
    servo's own scale. Accumulated in FrictionDRBamActuator.compute; read and cleared here.
    joints: regex over the actuator's joint names; only those are summed ("" = all).
    """
    import re
    from mjlab_microduck.actuator.friction_dr_bam import FrictionDRBamActuator
    asset = env.scene[asset_cfg.name]
    acts = [a for v in vars(asset).values()
            for a in (v.values() if isinstance(v, dict) else v if isinstance(v, (list, tuple)) else [])
            if isinstance(a, FrictionDRBamActuator)]
    act = acts[0]
    if act.tau_rate_stall <= 0.0:
        act.tau_rate_stall = stall_nm
        return torch.zeros(env.num_envs, device=env.device)
    if joints and not hasattr(act, "_tau_rate_mask"):
        names = list(getattr(act, "_target_names", None) or asset.joint_names)
        act._tau_rate_mask = torch.tensor([bool(re.search(joints, n)) for n in names], device=env.device)
    acc = act.tau_rate_acc
    out = (acc[:, act._tau_rate_mask] if joints else acc).sum(-1); act.tau_rate_acc.zero_()
    return out


def walk_progress(
    env: ManagerBasedRlEnv,
    command_name: str = "twist",
    vmax: float = 0.22,
    min_cmd: float = 0.02,
    head_gate: float = 0.0,
    asset_cfg: SceneEntityCfg = _ROBOT,
) -> torch.Tensor:
    """clamp(v · c/|c|, 0, min(|c|, vmax)) / vmax for |c| > min_cmd, else 0: linear pay for body speed along the
    commanded direction, capped at the command and at vmax (m/s).
    """
    asset = env.scene[asset_cfg.name]
    c = env.command_manager.get_command(command_name)[:, :2]
    n = torch.linalg.norm(c, dim=1)
    v = asset.data.root_link_lin_vel_b[:, :2]
    proj = (v * c).sum(dim=1) / torch.clamp(n, min=1e-6)
    r = torch.minimum(torch.clamp(proj, min=0.0), torch.clamp(n, max=vmax)) / vmax
    ema = getattr(env, "_head_bias_ema", None)              # head_pose_bias runs earlier in the same step
    if head_gate > 0.0 and ema is not None:
        nose_down = torch.clamp(ema[:, 1] - ema[:, 0], min=0.0)   # head_pitch + = nose down, neck_pitch − adds
        r = r * torch.clamp(1.0 - nose_down / head_gate, min=0.0)
    return torch.nan_to_num(r * (n > min_cmd).float(), nan=0.0)


def servo_motor_line_excess(
    env: ManagerBasedRlEnv,
    frac: float = 0.8,
    stall_nm: float = 1.37,
    w0: float = 10.7,
    asset_cfg: SceneEntityCfg = _ROBOT,
) -> torch.Tensor:
    """Sum over servos of relu(|tau| / tau_avail(omega) - frac)^2, tau_avail = stall * (1 - |omega| / w0).

    Load against what the HD-1910 motor can give at the joint's current speed (DC-motor line, 7.4 V:
    stall 1.37 N*m, no-load 10.7 rad/s), not against the static stall torque. Pair it with
    joint_pos_limit_proximity: a joint resting on its stop carries load through the constraint, which
    qfrc_actuator does not see.
    """
    asset = env.scene[asset_cfg.name]
    names = list(asset.joint_names)
    ids = [i for i, n in enumerate(names) if not n.startswith("passive_")]
    tau = asset.data.qfrc_actuator[:, ids].abs()
    avail = stall_nm * torch.clamp(1.0 - asset.data.joint_vel[:, ids].abs() / w0, min=0.05)
    over = torch.clamp(tau / avail - frac, min=0.0)
    return torch.nan_to_num(torch.sum(over * over, dim=1), nan=0.0)


# ── IMU mounting-misalignment ramp ──────────────────────────────────────────────────────────────────────────────
# Same per-env axis and angle distribution as the recipe (tasks/mdp.py::_imu_misalignment_quat: random axis,
# angle U[0, max], fixed per env), but the angle is multiplied by a factor growing linearly from f0 to 1 over
# <ramp> iterations from <it0>. Obs-level only; the critic keeps true values.
def _imu_ramped_quat(env: ManagerBasedRlEnv, max_angle_deg: float, it0: float, ramp: float, f0: float) -> torch.Tensor:
    from mjlab.utils.lab_api.math import quat_from_angle_axis
    import math
    if getattr(env, "_imu_ramp_axis", None) is None:
        n = env.num_envs
        axis = torch.randn(n, 3, device=env.device)
        env._imu_ramp_axis = axis / (torch.norm(axis, dim=-1, keepdim=True) + 1e-8)
        env._imu_ramp_angle = torch.rand(n, device=env.device) * math.radians(max_angle_deg)
    it = env.common_step_counter / 24.0
    f = f0 + (1.0 - f0) * min(max((it - it0) / max(ramp, 1.0), 0.0), 1.0)
    return quat_from_angle_axis(env._imu_ramp_angle * f, env._imu_ramp_axis)


def projected_gravity_imu_ramped(env: ManagerBasedRlEnv, max_angle_deg: float = 6.0, it0: float = 0.0, ramp: float = 1.0,
                                 f0: float = 1.0, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    from mjlab.utils.lab_api.math import quat_apply
    return quat_apply(_imu_ramped_quat(env, max_angle_deg, it0, ramp, f0), env.scene[asset_cfg.name].data.projected_gravity_b)


def base_ang_vel_imu_ramped(env: ManagerBasedRlEnv, max_angle_deg: float = 6.0, it0: float = 0.0, ramp: float = 1.0,
                            f0: float = 1.0, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    from mjlab.utils.lab_api.math import quat_apply
    return quat_apply(_imu_ramped_quat(env, max_angle_deg, it0, ramp, f0), env.scene[asset_cfg.name].data.root_link_ang_vel_b)


# ── Sustained servo current (HD-1910 datasheet: over-current protection trips at I > threshold for 2 s,
# factory 0.5 A, rated 0.9 A). Penalty only in the quiet upright stance. ──────────────────────────────────────────
def servo_current_hold(
    env: ManagerBasedRlEnv,
    i0_a: float = 0.5,
    kt: float = 0.692,
    tilt_max_deg: float = 60.0,
    settle_s: float = 0.5,
    v_max: float = 10.0,
    w_max: float = 100.0,
    z_min: float = 0.10,
    asset_cfg: SceneEntityCfg = _ROBOT,
) -> torch.Tensor:
    """Sum over servos of relu(|tau|/kt - i0)^2 [A^2] while the trunk is upright (tilt < tilt_max), quiet (|v| < v_max m/s,
    |w| < w_max rad/s) and the episode is older than settle_s; 0 otherwise."""
    asset = env.scene[asset_cfg.name]
    names = list(asset.joint_names)
    ids = [i for i, n in enumerate(names) if not n.startswith("passive_")]
    cur = asset.data.qfrc_actuator[:, ids].abs() / kt
    g = asset.data.projected_gravity_b
    tilt = torch.acos((-g[:, 2]).clamp(-1.0, 1.0))
    quiet = (asset.data.root_link_lin_vel_w.norm(dim=1) < v_max) & (asset.data.root_link_ang_vel_w.norm(dim=1) < w_max)
    # Gate = standing height (z > z_min) and not fallen (tilt < 60°).
    z = asset.data.root_link_pos_w[:, 2] - env.scene.env_origins[:, 2]
    gate = (tilt < math.radians(tilt_max_deg)) & (z > z_min) & quiet & (env.episode_length_buf.float() * env.step_dt > settle_s)
    over = torch.clamp(cur - i0_a, min=0.0)
    return torch.nan_to_num(torch.sum(over * over, dim=1) * gate.float(), nan=0.0)


def neck_strike_approach(env: ManagerBasedRlEnv, n0: float = 0.0, scale: float = 0.15, asset_cfg: SceneEntityCfg = _ROBOT) -> torch.Tensor:
    """((neck_pitch - HOME - n0)+ / scale)^2: quadratic, zero at and below HOME + n0 (rad)."""
    asset = env.scene[asset_cfg.name]
    if not hasattr(env, "_neck_id"):
        env._neck_id = asset.joint_names.index("neck_pitch")
    d = asset.data.joint_pos[:, env._neck_id] - asset.data.default_joint_pos[:, env._neck_id] - n0
    return torch.nan_to_num((d.clamp(min=0.0) / scale) ** 2, nan=0.0)


# ── Head/neck-only smoothing and head tracking only while standing ──────────────────────────────────────────────
# Head/neck-only action rate and torque rate; head tracking only once standing.
_HN = ("neck_pitch", "head_pitch", "head_yaw", "head_roll")


def _hn_action_ids(env, asset):
    if not hasattr(env, "_hn_aids"):
        servo = [n for n in asset.joint_names if not n.startswith("passive_")]
        env._hn_aids = [servo.index(n) for n in _HN]
        env._hn_jids = [asset.joint_names.index(n) for n in _HN]
    return env._hn_aids


def head_neck_action_rate_l2(env: ManagerBasedRlEnv, asset_cfg: SceneEntityCfg = _ROBOT) -> torch.Tensor:
    """Sum over neck/head of (a_t - a_{t-1})^2 (raw policy actions)."""
    ids = _hn_action_ids(env, env.scene[asset_cfg.name])
    am = env.action_manager
    return torch.sum(torch.square(am.action[:, ids] - am.prev_action[:, ids]), dim=1)


def head_neck_torque_rate_l2(env: ManagerBasedRlEnv, asset_cfg: SceneEntityCfg = _ROBOT) -> torch.Tensor:
    """Sum over neck/head of (tau_t - tau_{t-1})^2 [N*m]^2, own previous-step buffer."""
    asset = env.scene[asset_cfg.name]
    _hn_action_ids(env, asset)
    cur = asset.data.qfrc_actuator[:, env._hn_jids]
    prev = getattr(env, "_hn_prev_tau", None)
    env._hn_prev_tau = cur.clone()
    if prev is None or prev.shape != cur.shape:
        return torch.zeros(cur.shape[0], device=cur.device)
    fresh = env.episode_length_buf <= 1
    return torch.where(fresh, torch.zeros_like(cur[:, 0]), torch.sum(torch.square(cur - prev), dim=1))


def head_pose_tracking_standing(env: ManagerBasedRlEnv, command_name: str = "head_pose", std: float = 0.5,
                                z_min: float = 0.10, tilt_max_deg: float = 30.0, asset_cfg: SceneEntityCfg = _ROBOT) -> torch.Tensor:
    """The recipe's head_pose_tracking, paid only while standing (trunk z > z_min above the terrain, tilt < tilt_max):
    during the rise the head is free to balance."""
    from mjlab_microduck.tasks import mdp as _mmdp
    asset = env.scene[asset_cfg.name]
    z = asset.data.root_link_pos_w[:, 2] - env.scene.env_origins[:, 2]
    g = asset.data.projected_gravity_b
    tilt = torch.rad2deg(torch.acos((-g[:, 2]).clamp(-1.0, 1.0)))
    gate = ((z > z_min) & (tilt < tilt_max_deg)).float()
    return _mmdp.head_pose_tracking(env, command_name=command_name, std=std) * gate



def camera_stabilization(env: ManagerBasedRlEnv, thresh: float = 1.0, command_name: str = "twist", gate_std: float = 0.0,
                         asset_cfg: SceneEntityCfg = _ROBOT) -> torch.Tensor:
    """relu(roll² + yaw² − thresh): camera (site head_camera) angular rate in the world, rad²/s²: roll about the lens
    axis, yaw about world z minus the commanded turn rate. Only the excess over thresh is priced."""
    import mujoco
    if not hasattr(env, "_cam_stab_ids"):
        mj = env.sim.mj_model
        sid = mujoco.mj_name2id(mj, mujoco.mjtObj.mjOBJ_SITE, "robot/head_camera")
        bname = mujoco.mj_id2name(mj, mujoco.mjtObj.mjOBJ_BODY, int(mj.site_bodyid[sid])).split("/")[-1]
        env._cam_stab_ids = (sid, list(env.scene[asset_cfg.name].body_names).index(bname))
    sid, bi = env._cam_stab_ids
    asset = env.scene[asset_cfg.name]
    w = asset.data.body_link_ang_vel_w[:, bi]
    x = env.sim.data.site_xmat[:, sid].reshape(-1, 3, 3)[:, :, 0]
    roll = (w * x).sum(-1)
    yaw = w[:, 2] - env.command_manager.get_command(command_name)[:, 2]
    cost = torch.clamp(roll ** 2 + yaw ** 2 - thresh, min=0.0)
    if gate_std > 0.0:
        # gate_std > 0: price only a head that has arrived, gate exp(−|q_head − (HOME + command)|² / std²); while
        # turning to a new command the head is free.
        from mjlab_microduck.tasks import mdp as _mdp
        if not hasattr(env, "_head_pose_neck_ids"):
            _mdp.head_pose_tracking(env, command_name="head_pose", asset_cfg=asset_cfg)
        ids = env._head_pose_neck_ids
        err = asset.data.joint_pos[:, ids] - asset.data.default_joint_pos[:, ids] - env.command_manager.get_command("head_pose")
        cost = cost * torch.exp(-(err ** 2).sum(-1) / gate_std ** 2)
    return cost
