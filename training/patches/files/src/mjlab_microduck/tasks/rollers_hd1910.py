"""Roller-skate reward terms, commands and events for the HD-1910 duck."""

import math
import os
import torch
from dataclasses import dataclass
from mjlab.managers.command_manager import CommandTerm, CommandTermCfg

from mjlab.envs import ManagerBasedRlEnv


def body_speed_reward(env: ManagerBasedRlEnv, command_name: str = "twist", vel_scale: float = 0.3,
                      sensor_name: str = "feet_ground_contact") -> torch.Tensor:
    """clamp(throttle, 0) x tanh(forward trunk speed / vel_scale), while at least one blade is on the ground.

    Replaces Pollen's wheel_speed_reward (mean wheel spin): pays the trunk's own forward speed (heading frame),
    so wheels spinning in the air earn nothing."""
    cmd_x = env.command_manager.get_command(command_name)[:, 0]
    asset = env.scene["robot"]
    w, x, y, z = asset.data.root_link_quat_w.unbind(-1)
    yaw = torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    v = asset.data.root_link_lin_vel_w[:, :2]
    v_fwd = v[:, 0] * torch.cos(yaw) + v[:, 1] * torch.sin(yaw)
    found = env.scene.sensors[sensor_name].data.found
    grounded = (found.reshape(found.shape[0], -1).sum(-1) > 0).float()
    return torch.nan_to_num(torch.clamp(cmd_x, min=0.0) * torch.tanh(torch.clamp(v_fwd, min=0.0) / vel_scale) * grounded, nan=0.0)


def rolling_speed_reward(env: ManagerBasedRlEnv, command_name: str = "twist", vel_scale: float = 0.3,
                         wheel_radius: float = 0.015, sensor_name: str = "feet_ground_contact",
                         slip_std: float = 0.1, bidirectional: bool = False) -> torch.Tensor:
    """clamp(throttle, 0) x tanh(min(rolling speed of the GROUNDED wheels, forward trunk speed) / vel_scale).

    Only wheels whose blade is on the ground count, and the pay is capped by the trunk speed: moving
    forward by rolling is the only way to earn it."""
    cmd_x = env.command_manager.get_command(command_name)[:, 0]
    asset = env.scene["robot"]
    ids = {k: asset.find_joints(p)[0][0] for k, p in (("LF", "passive_LF_?wheel"), ("LR", "passive_LR_?wheel"),
                                                      ("RF", "passive_RF_?wheel"), ("RR", "passive_RR_?wheel"))}
    vel = asset.data.joint_vel
    found = env.scene.sensors[sensor_name].data.found.reshape(vel.shape[0], 2, -1).sum(-1) > 0   # (N, 2): left, right
    wl = (vel[:, ids["LF"]] + vel[:, ids["LR"]]) / 2.0
    wr = (vel[:, ids["RF"]] + vel[:, ids["RR"]]) / 2.0
    n = found.float().sum(-1)
    v_roll = (wl * found[:, 0].float() + wr * found[:, 1].float()) / n.clamp(min=1.0) * wheel_radius
    w, x, y, z = asset.data.root_link_quat_w.unbind(-1)
    yaw = torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    v = asset.data.root_link_lin_vel_w[:, :2]
    v_fwd = v[:, 0] * torch.cos(yaw) + v[:, 1] * torch.sin(yaw)
    # bidirectional (throttle < 0 = go backward): rolling and trunk speed taken in the commanded direction.
    sgn = torch.where(cmd_x < 0, -torch.ones_like(cmd_x), torch.ones_like(cmd_x)) if bidirectional else torch.ones_like(cmd_x)
    speed = torch.minimum(sgn * v_roll, sgn * v_fwd) * (n > 0).float()
    # Rolling consistency per grounded blade: its speed along the blade must match the rim speed of its wheels
    # (no slip -> ~1; wheels spun up in the air and landing faster than the ground -> ~0).
    if slip_std > 0:
        xa, jids, ank = _wheel_axes_world(env, asset)                             # (N, njnt, 3) hinge axes in world
        up = torch.tensor([0.0, 0.0, 1.0], device=xa.device).expand(xa.shape[0], 3)
        slip_c = torch.zeros_like(cmd_x)
        for i, wv in enumerate((wl, wr)):
            d = torch.linalg.cross(xa[:, jids[i]], up)                            # rolling direction
            d = d / d.norm(dim=-1, keepdim=True).clamp(min=1e-6)
            v_blade = (asset.data.body_link_lin_vel_w[:, ank[i]] * d).sum(-1)
            c = torch.exp(-((v_blade - wv * wheel_radius) / slip_std) ** 2)
            slip_c = slip_c + c * found[:, i].float()
        speed = speed * slip_c / n.clamp(min=1.0)
    gain = cmd_x.abs() if bidirectional else torch.clamp(cmd_x, min=0.0)
    return torch.nan_to_num(gain * torch.tanh(torch.clamp(speed, min=0.0) / vel_scale), nan=0.0)


def _wheel_axes_world(env: ManagerBasedRlEnv, asset):
    """World hinge axes of all joints as a torch view of the sim data, plus the front-wheel joint ids (left, right) and
    the ankle body ids (left, right) of the robot."""
    xa = env.sim.data.xaxis
    xa = xa if torch.is_tensor(xa) else xa._tensor                               # mjlab TorchArray: shared-memory torch view
    if not hasattr(env, "_rl_wheel_jids"):
        import mujoco
        m = env.sim.mj_model
        env._rl_wheel_jids = [next(j for j in range(m.njnt) if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) or "").endswith(n))
                              for n in ("passive_LF_wheel", "passive_RF_wheel")]
        env._rl_ankle_ids = [asset.find_bodies(b)[0][0] for b in ("ankle_l_v1", "ankle_r_v1")]
    return xa, env._rl_wheel_jids, env._rl_ankle_ids


from mjlab_microduck.tasks.mdp import RelativeHeadingVelocityCommand, RelativeHeadingVelocityCommandCfg  # noqa: E402


class SpawnHeadingVelocityCommand(RelativeHeadingVelocityCommand):
    """Pollen's relative-heading command with the target heading = the heading at episode start (kept until the next
    reset), so cmd[2] shows the drift that heading_hold_reward pays for."""

    def _resample_command(self, env_ids: torch.Tensor) -> None:
        keep = self._target_heading_w[env_ids].clone()
        super()._resample_command(env_ids)
        self._target_heading_w[env_ids] = keep                      # the target changes only at episode start
        if not hasattr(self, "_capture"):
            self._capture = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        # At reset the pose written by the reset event is not in the data yet; capture the heading on the next update.
        self._capture[env_ids] |= self._env.episode_length_buf[env_ids] <= 1

    def _update_command(self) -> None:
        if hasattr(self, "_capture") and self._capture.any():
            q = self.robot.data.root_link_quat_w
            w, x, y, z = q.unbind(-1)
            yaw = torch.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            self._target_heading_w = torch.where(self._capture, yaw, self._target_heading_w)
            self._capture[:] = False
        super()._update_command()


class SpawnHeadingVelocityCommandCfg(RelativeHeadingVelocityCommandCfg):
    def build(self, env: ManagerBasedRlEnv) -> "SpawnHeadingVelocityCommand":
        return SpawnHeadingVelocityCommand(self, env)


# ---- Braking recipe --------------------------------------------------------------------------------------------------

def _fwd_speed(asset):
    w, x, y, z = asset.data.root_link_quat_w.unbind(-1)
    yaw = torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    v = asset.data.root_link_lin_vel_w[:, :2]
    return v[:, 0] * torch.cos(yaw) + v[:, 1] * torch.sin(yaw)


def brake_progress(env: ManagerBasedRlEnv, v_cap: float = 0.6) -> torch.Tensor:
    """Potential-based: the drop of |forward trunk speed| since the last step (m/s per step, clamped so a speed-up costs and
    a stop from v0 pays exactly v0 over the episode)."""
    asset = env.scene["robot"]
    v = _fwd_speed(asset).abs().clamp(max=v_cap)
    if not hasattr(env, "_bk_prev") or env._bk_prev.shape[0] != v.shape[0]:
        env._bk_prev = v.clone()
    d = env._bk_prev - v
    env._bk_prev = v.clone()
    just_reset = env.episode_length_buf <= 1                          # after a reset the previous speed is another episode's
    return torch.where(just_reset, torch.zeros_like(d), d)


def stopped_upright(env: ManagerBasedRlEnv, v_stop: float = 0.03, sensor_name: str = "feet_ground_contact",
                    tilt_max: float = 0.175) -> torch.Tensor:
    """1 when the trunk is (nearly) still, both blades on the floor and the trunk within tilt_max of vertical, per step."""
    asset = env.scene["robot"]
    v = asset.data.root_link_lin_vel_w[:, :2].norm(dim=-1)
    found = env.scene.sensors[sensor_name].data.found
    both = (found.reshape(found.shape[0], 2, -1).sum(-1) > 0).all(dim=-1)
    g = asset.data.projected_gravity_b
    tilt = torch.acos((-g[:, 2]).clamp(-1, 1))
    return ((v < v_stop) & both & (tilt < tilt_max)).float()


# ---- A-swizzle recipe ------------------------------------------------------------------------------------------------

def _rolling_speed(env: ManagerBasedRlEnv, asset, sensor_name: str, wheel_radius: float, slip_std: float):
    """Signed forward speed 'by rolling': min over (rim speed of the grounded blades, trunk speed) taken along the trunk
    heading, times the rolling-consistency factor of the grounded blades (as in rolling_speed_reward). Returns
    (speed, n_grounded, v_fwd)."""
    ids = {k: asset.find_joints(p)[0][0] for k, p in (("LF", "passive_LF_?wheel"), ("LR", "passive_LR_?wheel"),
                                                      ("RF", "passive_RF_?wheel"), ("RR", "passive_RR_?wheel"))}
    vel = asset.data.joint_vel
    found = env.scene.sensors[sensor_name].data.found.reshape(vel.shape[0], 2, -1).sum(-1) > 0
    wl = (vel[:, ids["LF"]] + vel[:, ids["LR"]]) / 2.0
    wr = (vel[:, ids["RF"]] + vel[:, ids["RR"]]) / 2.0
    n = found.float().sum(-1)
    v_roll = (wl * found[:, 0].float() + wr * found[:, 1].float()) / n.clamp(min=1.0) * wheel_radius
    w, x, y, z = asset.data.root_link_quat_w.unbind(-1)
    yaw = torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    v = asset.data.root_link_lin_vel_w[:, :2]
    v_fwd = v[:, 0] * torch.cos(yaw) + v[:, 1] * torch.sin(yaw)
    sgn = torch.sign(v_fwd) + (v_fwd == 0).float()
    speed = sgn * torch.clamp(torch.minimum(sgn * v_roll, sgn * v_fwd), min=0.0)   # same sign as the trunk, never beyond it
    slip_c = torch.zeros_like(v_fwd)
    xa, jids, ank = _wheel_axes_world(env, asset)
    up = torch.tensor([0.0, 0.0, 1.0], device=xa.device).expand(xa.shape[0], 3)
    for i, wv in enumerate((wl, wr)):
        d = torch.linalg.cross(xa[:, jids[i]], up)
        d = d / d.norm(dim=-1, keepdim=True).clamp(min=1e-6)
        v_blade = (asset.data.body_link_lin_vel_w[:, ank[i]] * d).sum(-1)
        slip_c = slip_c + torch.exp(-((v_blade - wv * wheel_radius) / slip_std) ** 2) * found[:, i].float()
    speed = speed * slip_c / n.clamp(min=1.0) * (n > 0).float()
    return speed, n, v_fwd


def rolling_speed_track(env: ManagerBasedRlEnv, command_name: str = "twist", std: float = 0.15, wheel_radius: float = 0.015,
                        sensor_name: str = "feet_ground_contact", slip_std: float = 0.1) -> torch.Tensor:
    """exp(-((v_cmd - v)/std)^2): the commanded speed (cmd[0], m/s, signed) is tracked by the speed 'by rolling': the
    grounded wheels must turn with the ground (no skid), and they cannot earn more than the trunk moves (no spin in the
    air). cmd 0 pays for standing still."""
    cmd = env.command_manager.get_command(command_name)[:, 0]
    asset = env.scene["robot"]
    speed, n, v_fwd = _rolling_speed(env, asset, sensor_name, wheel_radius, slip_std)
    v = torch.where(n > 0, speed, v_fwd)                                          # no blade down: the trunk speed (no credit for rolling)
    return torch.nan_to_num(torch.exp(-((cmd - v) / std) ** 2), nan=0.0)


def both_blades_grounded(env: ManagerBasedRlEnv, sensor_name: str = "feet_ground_contact") -> torch.Tensor:
    """1 when both blades touch the floor, else 0. Not scaled by the command."""
    found = env.scene.sensors[sensor_name].data.found
    return (found.reshape(found.shape[0], 2, -1).sum(-1) > 0).all(dim=-1).float()


def blade_pitch_penalty(env: ManagerBasedRlEnv, sensor_name: str = "feet_ground_contact") -> torch.Tensor:
    """sin^2 of each grounded blade's fore-aft tilt (toe up/down), summed. The sideways tilt from hip_roll is free
    (the ankle has no roll joint)."""
    asset = env.scene["robot"]
    xa, jids, ank = _wheel_axes_world(env, asset)
    up = torch.tensor([0.0, 0.0, 1.0], device=xa.device).expand(xa.shape[0], 3)
    found = env.scene.sensors[sensor_name].data.found.reshape(xa.shape[0], 2, -1).sum(-1) > 0
    out = torch.zeros(xa.shape[0], device=xa.device)
    for i in range(2):
        d = torch.linalg.cross(xa[:, jids[i]], up)
        d = d / d.norm(dim=-1, keepdim=True).clamp(min=1e-6)
        out = out + d[:, 2] ** 2 * found[:, i].float()
    return out


# ---- Glide / slope recipe -------------------------------------------------------------------------------------------

def rolling_speed_capped(env: ManagerBasedRlEnv, cap: float = 0.35, wheel_radius: float = 0.015,
                         sensor_name: str = "feet_ground_contact", slip_std: float = 0.1) -> torch.Tensor:
    """tanh(speed by rolling / cap): pays for rolling down (forward), saturating at the cap, no penalty above it;
    the same 'by rolling' speed as the ride (grounded wheels, capped by the trunk, slip factor)."""
    asset = env.scene["robot"]
    speed, n, _ = _rolling_speed(env, asset, sensor_name, wheel_radius, slip_std)
    return torch.nan_to_num(torch.tanh(speed.abs() / cap), nan=0.0)          # either direction (glide backward)


def glide_quiet_legs(env: ManagerBasedRlEnv, std: float = 2.0, v_min: float = 0.05,
                     sensor_name: str = "feet_ground_contact") -> torch.Tensor:
    """exp(-mean(leg joint speed^2)/std^2) while rolling faster than v_min on both blades: legs still, the wheels do
    the work. Zero when standing or with a blade in the air."""
    asset = env.scene["robot"]
    if not hasattr(env, "_gl_leg_ids"):
        env._gl_leg_ids = [i for i, n in enumerate(asset.joint_names) if not n.startswith("passive_") and any(k in n for k in ("hip", "knee", "ankle"))]
    qd = asset.data.joint_vel[:, env._gl_leg_ids]
    quiet = torch.exp(-(qd ** 2).mean(dim=-1) / std ** 2)
    v = _fwd_speed(asset).abs()
    found = env.scene.sensors[sensor_name].data.found
    both = (found.reshape(found.shape[0], 2, -1).sum(-1) > 0).all(dim=-1)
    return quiet * both.float() * (v > v_min).float()


def glide_quiet_tracking(env: ManagerBasedRlEnv, command_name: str = "twist", std: float = 2.0, speed_std: float = 0.25,
                         wheel_radius: float = 0.015, sensor_name: str = "feet_ground_contact", slip_std: float = 0.1) -> torch.Tensor:
    """glide_quiet_legs x rolling_speed_track: still legs pay only at the commanded speed."""
    return glide_quiet_legs(env, std=std, sensor_name=sensor_name) * rolling_speed_track(
        env, command_name=command_name, std=speed_std, wheel_radius=wheel_radius, sensor_name=sensor_name, slip_std=slip_std)


def rolling_quality_speed(env: ManagerBasedRlEnv, sensor_name: str = "feet_ground_contact", lat_std: float = 0.1) -> torch.Tensor:
    """Signed trunk speed along its heading x rolling quality: mean over the grounded blades of exp(-(sideways skid / lat_std)^2)
    (blade = its two wheel centres, skid = their mean velocity across the blade, m/s). 0 with no blade down."""
    asset = env.scene["robot"]
    if not hasattr(env, "_rq_w"):
        env._rq_w = [asset.find_bodies(n)[0][0] for n in ("tire", "tire_2", "tire_3", "tire_4")]
    P = asset.data.body_link_pos_w[:, env._rq_w, :2]; V = asset.data.body_link_lin_vel_w[:, env._rq_w, :2]
    found = env.scene.sensors[sensor_name].data.found.reshape(P.shape[0], 2, -1).sum(-1) > 0
    q = torch.zeros(P.shape[0], device=P.device)
    for s in range(2):
        ax = P[:, 2 * s] - P[:, 2 * s + 1]; ax = ax / ax.norm(dim=-1, keepdim=True).clamp(min=1e-6)
        perp = torch.stack([-ax[:, 1], ax[:, 0]], -1)
        lat = (0.5 * (V[:, 2 * s] + V[:, 2 * s + 1]) * perp).sum(-1)
        q = q + torch.exp(-(lat / lat_std) ** 2) * found[:, s].float()
    n = found.float().sum(-1)
    return _fwd_speed(asset) * q / n.clamp(min=1.0) * (n > 0).float()


def rolling_quality_track(env: ManagerBasedRlEnv, command_name: str = "twist", std: float = 0.15, lat_std: float = 0.1,
                          sensor_name: str = "feet_ground_contact") -> torch.Tensor:
    """exp(-((cmd - rolling_quality_speed)/std)^2)."""
    cmd = env.command_manager.get_command(command_name)[:, 0]
    v = rolling_quality_speed(env, sensor_name=sensor_name, lat_std=lat_std)
    return torch.nan_to_num(torch.exp(-((cmd - v) / std) ** 2), nan=0.0)


def glide_quiet_quality(env: ManagerBasedRlEnv, command_name: str = "twist", std: float = 2.0, speed_std: float = 0.15,
                        lat_std: float = 0.1, sensor_name: str = "feet_ground_contact") -> torch.Tensor:
    """glide_quiet_legs x rolling_quality_track: the coast pays only at the commanded speed, measured as the quality speed."""
    return glide_quiet_legs(env, std=std, sensor_name=sensor_name) * rolling_quality_track(
        env, command_name=command_name, std=speed_std, lat_std=lat_std, sensor_name=sensor_name)


# ---- Braking in both directions --------------------------------------------------------------------------------------

def reset_rolling_entry_bidir(env: ManagerBasedRlEnv, env_ids, speed_range=(0.2, 0.5), back_prob: float = 0.5) -> None:
    """Pollen's rolling entry with a random direction per env: half the envs roll backward (negative speed)."""
    from mjlab_microduck.tasks.mdp import reset_rolling_entry
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device)
    back = torch.rand(len(env_ids), device=env.device) < back_prob
    if (~back).any():
        reset_rolling_entry(env, env_ids[~back], speed_range=speed_range)
    if back.any():
        reset_rolling_entry(env, env_ids[back], speed_range=(-speed_range[1], -speed_range[0]))


def reset_from_state_bank(env: ManagerBasedRlEnv, env_ids, path: str, frac: float = 0.5) -> None:
    """`frac` of the starts from recorded states (a state bank .pt). Trunk at r_wheel + dz above the floor (dz = trunk
    above the lowest wheel centre), tilt and velocities as recorded (yaw removed at recording, matching the yaw-0 spawn),
    every joint and wheel, and the source's last action, written into the action buffer right after the action manager
    zeroes it (events run before that reset)."""
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device)
    if not hasattr(env, "_bank"):
        b = torch.load(path, map_location=env.device)
        env._bank = {k: (v.to(env.device) if torch.is_tensor(v) else v) for k, v in b.items()}
        env._bank_src = torch.full((env.num_envs,), -1, dtype=torch.long, device=env.device)
        env._bank_pending = None
        am = env.action_manager
        _orig = am.reset

        def _reset(ids=None):
            info = _orig(ids)
            p = env._bank_pending
            if p is not None:
                am._action[p[0]] = p[1]; am._prev_action[p[0]] = p[1]
                env._bank_pending = None
            return info
        am.reset = _reset
    B = env._bank
    env._bank_src[env_ids] = -1
    sel = env_ids[torch.rand(len(env_ids), device=env.device) < frac]
    if len(sel) == 0:
        return
    k = torch.randint(0, B["dz"].shape[0], (len(sel),), device=env.device)
    asset = env.scene["robot"]
    pos = env.scene.env_origins[sel].clone()
    pos[:, 2] += float(B["r_wheel"]) + B["dz"][k] + 0.0005
    asset.write_root_link_pose_to_sim(torch.cat([pos, B["quat"][k]], dim=-1), env_ids=sel)
    asset.write_root_link_velocity_to_sim(torch.cat([B["lin_vel"][k], B["ang_vel"][k]], dim=-1), env_ids=sel)
    asset.write_joint_state_to_sim(B["joint_pos"][k], B["joint_vel"][k], env_ids=sel)
    env._bank_pending = (sel, B["action"][k])
    env._bank_src[sel] = B["src"][k]


class BrakeDirCommand(SpawnHeadingVelocityCommand):
    """cmd[0] = +/-dir_mag: the direction the duck was rolling when the brake took over (captured on the first update after
    reset from the trunk speed), held for the episode. The actor has no trunk velocity in its observations, so this
    tells a backward roll from a forward one. The runtime writes the sign of the last ride command."""
    dir_mag = 0.3

    def _update_command(self) -> None:
        cap = getattr(self, "_capture", None)
        need = cap is not None and cap.any()
        if need:
            v = _fwd_speed(self.robot)
            d = torch.where(v >= 0, torch.ones_like(v), -torch.ones_like(v))
            if not hasattr(self, "_dir"):
                self._dir = torch.ones(self.num_envs, device=self.device)
            self._dir = torch.where(cap, d, self._dir)
        super()._update_command()
        if hasattr(self, "_dir"):
            self.vel_command_b[:, 0] = self.dir_mag * self._dir


class BrakeDirCommandCfg(SpawnHeadingVelocityCommandCfg):
    def build(self, env: ManagerBasedRlEnv) -> "BrakeDirCommand":
        return BrakeDirCommand(self, env)


def not_rolling_penalty(env: ManagerBasedRlEnv, v_min: float = 0.05) -> torch.Tensor:
    """1 while the trunk moves slower than v_min (use a negative weight): standing earns nothing from the positive
    per-step terms."""
    asset = env.scene["robot"]
    v = asset.data.root_link_lin_vel_w.norm(dim=-1)                             # full speed (along the surface on ramps)
    return (v < v_min).float()


def reset_rolling_entry_world(env: ManagerBasedRlEnv, env_ids, speed_range=(0.25, 0.45), wheel_sign: float = 1.0,
                              wheel_radius: float = 0.015) -> None:
    """Pollen's rolling entry (base velocity along world +x, wheels at v/r) with the wheel spin sign free: the backward glide
    spawns the duck facing -x (yaw 180°) and rolling down +x tail first, so its wheels turn backward (wheel_sign -1)."""
    asset = env.scene["robot"]
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device)
    n = int(env_ids.shape[0]); lo, hi = speed_range
    v = torch.rand(n, device=env.device) * (hi - lo) + lo
    root_vel = torch.zeros(n, 6, device=env.device); root_vel[:, 0] = v
    asset.write_root_link_velocity_to_sim(root_vel, env_ids=env_ids)
    wheel_ids = torch.tensor([asset.find_joints(p)[0][0] for p in ("passive_LF_?wheel", "passive_LR_?wheel", "passive_RF_?wheel", "passive_RR_?wheel")], device=env.device)
    omega = (wheel_sign * v / wheel_radius).unsqueeze(1).repeat(1, 4)
    asset.write_joint_velocity_to_sim(omega, joint_ids=wheel_ids, env_ids=env_ids)


# ---- Tricks: one-skate glide, spin, crab ----------------------------------------------------------------------------

def _foot_states(env, sensor_name="feet_ground_contact"):
    found = env.scene.sensors[sensor_name].data.found
    return found.reshape(found.shape[0], 2, -1).sum(-1) > 0                  # (N, 2) left, right on the floor


def one_skate_glide(env: ManagerBasedRlEnv, command_name: str = "twist", vel_scale: float = 0.3,
                    sensor_name: str = "feet_ground_contact", lift_full: float = 0.03, lift_min: float = 0.0,
                    swing_quiet_std: float = 0.0) -> torch.Tensor:
    """Glide on one skate: the commanded foot (cmd[1] > 0: lift the left, < 0: the right) is raised while the other rolls.
    Pays the lift continuously (0 -> 1 over lift_full) x stance skate on the floor x rolling (v > 0.05 m/s) x quiet
    stance leg. No credit for speed."""
    asset = env.scene["robot"]
    side = env.command_manager.get_command(command_name)[:, 1]
    on = _foot_states(env, sensor_name)
    if not hasattr(env, "_os_ank"):
        env._os_ank = [asset.find_bodies(b)[0][0] for b in ("ankle_l_v1", "ankle_r_v1")]
        names = asset.joint_names
        env._os_legs = [[i for i, n in enumerate(names) if n.startswith(pre) and any(k in n for k in ("hip", "knee", "ankle"))] for pre in ("left_", "right_")]
    z = asset.data.body_link_pos_w[:, env._os_ank, 2]
    left = side > 0
    lift = torch.where(left, z[:, 0] - z[:, 1], z[:, 1] - z[:, 0])
    lift_frac = ((lift - lift_min) / (lift_full - lift_min)).clamp(0.0, 1.0)    # nothing below lift_min
    swing_off = torch.where(left, ~on[:, 0], ~on[:, 1])
    if swing_quiet_std > 0:                                                      # the lifted foot must hang still (trunk frame)
        vf = asset.data.body_link_lin_vel_w[:, env._os_ank] - asset.data.root_link_lin_vel_w.unsqueeze(1)
        vsw = torch.where(left, vf[:, 0].norm(dim=-1), vf[:, 1].norm(dim=-1))
        lift_frac = lift_frac * torch.exp(-(vsw / swing_quiet_std) ** 2)
    stance_on = torch.where(left, on[:, 1], on[:, 0])
    qd = asset.data.joint_vel
    quiet_l = torch.exp(-(qd[:, env._os_legs[0]] ** 2).mean(dim=-1) / 2.0 ** 2)
    quiet_r = torch.exp(-(qd[:, env._os_legs[1]] ** 2).mean(dim=-1) / 2.0 ** 2)
    quiet_stance = torch.where(left, quiet_r, quiet_l)
    v = _fwd_speed(asset)
    return lift_frac * swing_off.float() * stance_on.float() * (v > 0.05).float() * quiet_stance

def yaw_rate_track(env: ManagerBasedRlEnv, command_name: str = "twist", std: float = 1.0) -> torch.Tensor:
    """exp(-((w_cmd - w_z)/std)^2): the spin — trunk yaw rate tracks cmd[2] (rad/s, signed)."""
    asset = env.scene["robot"]
    w = asset.data.root_link_ang_vel_w[:, 2]
    cmd = env.command_manager.get_command(command_name)[:, 2]
    return torch.exp(-((cmd - w) / std) ** 2)


def stay_in_place(env: ManagerBasedRlEnv) -> torch.Tensor:
    """|trunk horizontal speed|^2: the spin turns on the spot."""
    asset = env.scene["robot"]
    return (asset.data.root_link_lin_vel_w[:, :2] ** 2).sum(-1)


def lateral_rolling_track(env: ManagerBasedRlEnv, command_name: str = "twist", std: float = 0.08, wheel_radius: float = 0.015,
                          sensor_name: str = "feet_ground_contact", slip_std: float = 0.1, push: bool = False) -> torch.Tensor:
    """The crab: cmd[1] = lateral trunk speed (m/s, + = left). Tracked by the speed 'by rolling' along the trunk's y: rim speed
    of the grounded wheels projected on y, capped by the trunk's lateral speed, times the per-blade rolling consistency
    (the skates must be turned across the trunk for the wheels to roll sideways)."""
    cmd = env.command_manager.get_command(command_name)[:, 1]
    asset = env.scene["robot"]
    w, x, y, z = asset.data.root_link_quat_w.unbind(-1)
    yaw = torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    vw = asset.data.root_link_lin_vel_w[:, :2]
    v_lat = -vw[:, 0] * torch.sin(yaw) + vw[:, 1] * torch.cos(yaw)
    ids = {k: asset.find_joints(p)[0][0] for k, p in (("LF", "passive_LF_?wheel"), ("LR", "passive_LR_?wheel"),
                                                      ("RF", "passive_RF_?wheel"), ("RR", "passive_RR_?wheel"))}
    vel = asset.data.joint_vel
    found = _foot_states(env, sensor_name)
    wl = (vel[:, ids["LF"]] + vel[:, ids["LR"]]) / 2.0
    wr = (vel[:, ids["RF"]] + vel[:, ids["RR"]]) / 2.0
    n = found.float().sum(-1)
    xa, jids, ank = _wheel_axes_world(env, asset)
    up = torch.tensor([0.0, 0.0, 1.0], device=xa.device).expand(xa.shape[0], 3)
    lat = torch.stack([-torch.sin(yaw), torch.cos(yaw), torch.zeros_like(yaw)], -1)
    rim_lat = torch.zeros_like(v_lat); slip_c = torch.zeros_like(v_lat)
    for i, wv in enumerate((wl, wr)):
        d = torch.linalg.cross(xa[:, jids[i]], up)
        d = d / d.norm(dim=-1, keepdim=True).clamp(min=1e-6)
        v_blade = (asset.data.body_link_lin_vel_w[:, ank[i]] * d).sum(-1)
        slip_c = slip_c + torch.exp(-((v_blade - wv * wheel_radius) / slip_std) ** 2) * found[:, i].float()
        rim_lat = rim_lat + (wv * wheel_radius) * (d * lat).sum(-1) * found[:, i].float()   # rim speed projected on the trunk's y
    rim_lat = rim_lat / n.clamp(min=1.0)
    sgn = torch.sign(v_lat) + (v_lat == 0).float()
    speed = sgn * torch.clamp(torch.minimum(sgn * rim_lat, sgn * v_lat), min=0.0) * slip_c / n.clamp(min=1.0) * (n > 0).float()
    if push:                                                                   # discovery: |cmd| x tanh(lateral rolling in the commanded direction / 0.15)
        return torch.nan_to_num(cmd.abs() * torch.tanh(torch.clamp(torch.sign(cmd) * speed, min=0.0) / 0.15), nan=0.0)
    return torch.nan_to_num(torch.exp(-((cmd - speed) / std) ** 2), nan=0.0)


class OneSkateSideCommand(SpawnHeadingVelocityCommand):
    """cmd[1] = +1 (lift the left skate) or -1 (the right), held for the episode; cmd[2] heading error as usual."""
    def _resample_command(self, env_ids: torch.Tensor) -> None:
        super()._resample_command(env_ids)
        self.vel_command_b[env_ids, 1] = torch.where(torch.rand(len(env_ids), device=self.device) < 0.5, -1.0, 1.0)

    def _update_command(self) -> None:
        keep = self.vel_command_b[:, 1].clone()
        super()._update_command()
        self.vel_command_b[:, 1] = keep
        _st = os.environ.get("MICRODUCK_ONESKATE_STRIDE")                   # MICRODUCK_ONESKATE_STRIDE: the side flips every T_half s
        if _st:
            t_lo, t_hi = (float(x) for x in _st.split(",")[:2])
            n = keep.shape[0]
            if not hasattr(self, "_st_half") or self._st_half.shape[0] != n:
                self._st_half = t_lo + (t_hi - t_lo) * torch.rand(n, device=self.device)
                self._st_side0 = torch.where(torch.rand(n, device=self.device) < 0.5, -1.0, 1.0)
            t = self._env.episode_length_buf.float() * self._env.step_dt
            new_ep = self._env.episode_length_buf <= 1
            self._st_half = torch.where(new_ep, t_lo + (t_hi - t_lo) * torch.rand(n, device=self.device), self._st_half)
            self._st_side0 = torch.where(new_ep, torch.where(torch.rand(n, device=self.device) < 0.5, -1.0, 1.0), self._st_side0)
            k = torch.floor(t / self._st_half)
            self.vel_command_b[:, 1] = self._st_side0 * torch.where(k % 2 == 0, 1.0, -1.0)
            return
        hold = float(os.environ.get("MICRODUCK_ONESKATE_END_HOLD", "0"))   # MICRODUCK_ONESKATE_END_HOLD: last `hold` s -> cmd[1] = 0: put the skate down
        if hold > 0:
            t_left = self._env.max_episode_length_s - self._env.episode_length_buf.float() * self._env.step_dt
            if not hasattr(self, "_side_keep") or self._side_keep.shape[0] != keep.shape[0]:
                self._side_keep = keep.clone()
            self._side_keep = torch.where(keep != 0, keep, self._side_keep)
            self.vel_command_b[:, 1] = torch.where(t_left < hold, torch.zeros_like(keep), self._side_keep)


class OneSkateSideCommandCfg(SpawnHeadingVelocityCommandCfg):
    def build(self, env: ManagerBasedRlEnv) -> "OneSkateSideCommand":
        return OneSkateSideCommand(self, env)


class SpinCommand(RelativeHeadingVelocityCommand):
    """cmd[2] = yaw-rate target: sign random, magnitude 1..3 rad/s, held for the episode (no heading error here)."""
    def _resample_command(self, env_ids: torch.Tensor) -> None:
        super()._resample_command(env_ids)
        n = len(env_ids)
        mag = 1.0 + 2.0 * torch.rand(n, device=self.device)
        self.vel_command_b[env_ids, 2] = torch.where(torch.rand(n, device=self.device) < 0.5, -mag, mag)

    def _update_command(self) -> None:
        pass


class SpinCommandCfg(RelativeHeadingVelocityCommandCfg):
    def build(self, env: ManagerBasedRlEnv) -> "SpinCommand":
        return SpinCommand(self, env)


class CrabCommand(SpawnHeadingVelocityCommand):
    """cmd[1] = lateral speed with |cmd| in (lo, hi), never near zero; sign random; cmd[2] heading error vs spawn."""
    lo, hi = 0.12, 0.25

    def _resample_command(self, env_ids: torch.Tensor) -> None:
        super()._resample_command(env_ids)
        n = len(env_ids)
        mag = self.lo + (self.hi - self.lo) * torch.rand(n, device=self.device)
        self.vel_command_b[env_ids, 1] = torch.where(torch.rand(n, device=self.device) < 0.5, -mag, mag)


class CrabCommandCfg(SpawnHeadingVelocityCommandCfg):
    def build(self, env: ManagerBasedRlEnv) -> "CrabCommand":
        return CrabCommand(self, env)


def terrain_levels_glide(env: ManagerBasedRlEnv, env_ids: torch.Tensor, min_distance: float = 2.5, max_row: int = 0) -> torch.Tensor:
    """Ramp curriculum tied to safety, not distance: promote a duck that ended its episode upright (no fell_over) after at
    least min_distance down the tile; demote one that fell."""
    asset = env.scene["robot"]
    terrain = env.scene.terrain
    assert terrain is not None
    distance = asset.data.root_link_pos_w[env_ids, 0] - env.scene.env_origins[env_ids, 0]
    fell = env.termination_manager.get_term("fell_over")[env_ids]
    move_up = ~fell & (distance > min_distance)
    move_down = fell
    if max_row:                                                           # never promote beyond max_row
        move_up = move_up & (terrain.terrain_levels[env_ids] < max_row)
    terrain.update_env_origins(env_ids, move_up, move_down)
    return torch.mean(terrain.terrain_levels.float())


# ---- Slope: on-ramp spawn pose + speed ceiling ----------------------------------------------------------------------

def reset_on_ramp_pose(env: ManagerBasedRlEnv, env_ids, spawn_on_ramp: float = 0.3, stand_z: float = 0.1405, lift: float = 0.005,
                       yaw: float = 0.0) -> None:
    """Ducks spawned on the ramp (origin z < 0, the 'flat_ramp_on' column): tilt the trunk to the ramp (rotation about world y
    by the ramp angle, so a duck facing +x gets its nose down and one facing -x (glide backward) its tail down) and put it
    stand_z*cos(angle)+lift above the inclined surface. angle = atan(-origin_z / spawn_on_ramp); the spawn is
    spawn_on_ramp metres down the ramp."""
    asset = env.scene["robot"]
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device)
    oz = env.scene.env_origins[env_ids, 2]
    on = oz < -1e-4
    if not on.any():
        return
    ids = env_ids[on]
    a = torch.atan2(-oz[on], torch.full_like(oz[on], spawn_on_ramp))
    # the entity data is stale inside a reset (it still holds the previous episode's pose), so build the pose from the env
    # origin and the recipe's spawn yaw, not from asset.data
    pos = env.scene.env_origins[ids].clone()
    pos[:, 2] = pos[:, 2] + stand_z * torch.cos(a) + lift
    hy = torch.full_like(a, yaw / 2.0)
    q = torch.stack([torch.cos(hy), torch.zeros_like(hy), torch.zeros_like(hy), torch.sin(hy)], -1)  # yaw about z
    h = a / 2.0
    qp = torch.stack([torch.cos(h), torch.zeros_like(h), torch.sin(h), torch.zeros_like(h)], -1)    # (w, x, y, z), about world y
    w1, x1, y1, z1 = qp.unbind(-1); w2, x2, y2, z2 = q.unbind(-1)
    qn = torch.stack([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                      w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2], -1)
    asset.write_root_link_pose_to_sim(torch.cat([pos, qn], -1), env_ids=ids)


def overspeed_penalty(env: ManagerBasedRlEnv, v_max: float = 0.7) -> torch.Tensor:
    """Speed ceiling for the glide: the excess of the trunk speed over v_max, per step (m/s). Linear."""
    asset = env.scene["robot"]
    v = asset.data.root_link_lin_vel_w.norm(dim=-1)                             # full speed (along the surface)
    return (v - v_max).clamp(min=0.0)


# ---- Stride ---------------------------------------------------------------------------------------------------------

class StridePhaseCommand(CommandTerm):
    """Gait clock for the skating stride: command = (sin phi, cos phi), phi = 2*pi*(t + offset)/period with t the episode time.
    sin > 0: the LEFT skate is the free (lifted) one and the right one rolls; sin < 0: the right is free. A random offset per
    episode so the stride is not tied to the reset. Observed by the policy (generated_commands) like the head command."""

    cfg: "StridePhaseCommandCfg"

    def __init__(self, cfg: "StridePhaseCommandCfg", env: ManagerBasedRlEnv):
        super().__init__(cfg, env)
        self._command = torch.zeros(self.num_envs, 2, device=self.device)
        self._offset = torch.zeros(self.num_envs, device=self.device)
        self._env = env

    @property
    def command(self) -> torch.Tensor:
        return self._command

    def _update_metrics(self) -> None:
        pass

    def _resample_command(self, env_ids: torch.Tensor) -> None:
        if not hasattr(self, "_period"):
            self._period = torch.full((self.num_envs,), self.cfg.period, device=self.device)
        if len(env_ids):
            if self.cfg.period_range is not None:                        # fish: a period per episode
                lo, hi = self.cfg.period_range
                self._period[env_ids] = lo + (hi - lo) * torch.rand(len(env_ids), device=self.device)
            self._offset[env_ids] = torch.rand(len(env_ids), device=self.device) * self._period[env_ids]

    def _update_command(self) -> None:
        if not hasattr(self, "_period"):
            self._period = torch.full((self.num_envs,), self.cfg.period, device=self.device)
        t = self._env.episode_length_buf.float() * self._env.step_dt + self._offset
        phi = 2.0 * math.pi * t / self._period
        self._command[:, 0] = torch.sin(phi); self._command[:, 1] = torch.cos(phi)


@dataclass(kw_only=True)
class StridePhaseCommandCfg(CommandTermCfg):
    period: float = 1.2                                                  # s per full stride (both feet)
    period_range: tuple[float, float] | None = None                      # fish: sampled per episode

    def build(self, env: ManagerBasedRlEnv) -> "StridePhaseCommand":
        return StridePhaseCommand(self, env)


class SwizzleClockCommand(CommandTerm):
    """Swizzle clock: command = (sin phi, cos phi), phi integrated with f = |cmd speed| / (gain * amp),
    capped at f_max, frozen below v_min (standing). gain from the scripted swizzle in ideal sim (toe 10 deg, both skates
    down, no policy): v ~ 4.1 * A * f m/s."""

    cfg: "SwizzleClockCommandCfg"

    def __init__(self, cfg: "SwizzleClockCommandCfg", env: ManagerBasedRlEnv):
        super().__init__(cfg, env)
        self._command = torch.zeros(self.num_envs, 2, device=self.device)
        self._phi = torch.zeros(self.num_envs, device=self.device)
        self._env = env

    @property
    def command(self) -> torch.Tensor:
        return self._command

    def _update_metrics(self) -> None:
        pass

    def _resample_command(self, env_ids: torch.Tensor) -> None:
        if len(env_ids):
            self._phi[env_ids] = torch.rand(len(env_ids), device=self.device) * 2.0 * math.pi

    def _update_command(self) -> None:
        v = self._env.command_manager.get_command(self.cfg.speed_command)[:, 0].abs()
        f = (v / (self.cfg.gain * self.cfg.amp)).clamp(max=self.cfg.f_max) * (v > self.cfg.v_min).float()
        self._phi = torch.remainder(self._phi + 2.0 * math.pi * f * self._env.step_dt, 2.0 * math.pi)
        self._command[:, 0] = torch.sin(self._phi); self._command[:, 1] = torch.cos(self._phi)


@dataclass(kw_only=True)
class SwizzleClockCommandCfg(CommandTermCfg):
    speed_command: str = "twist"
    amp: float = 0.15                                                    # hip-roll amplitude, rad
    gain: float = 4.1                                                    # v = gain * amp * f (scripted swizzle, toe 10 deg)
    f_max: float = 1.5                                                   # Hz, stable range of the scripted swizzle
    v_min: float = 0.05

    def build(self, env: ManagerBasedRlEnv) -> "SwizzleClockCommand":
        return SwizzleClockCommand(self, env)


def swizzle_reference(env: ManagerBasedRlEnv, clock: str = "swizzle_phase", command_name: str = "twist", amp: float = 0.15,
                      toe: float = 0.1745, v_min: float = 0.05):
    """Joint ids (L roll, R roll, L yaw, R yaw) and their swizzle targets: hip roll HOME + out * amp * sin(phi) (legs apart/
    together), hip yaw HOME + toe-out * sign(cmd) * toe * cos(phi) (toes out while spreading forward; heels out backward).
    Signs: left roll + / yaw - = outward / toe-out, right mirrored. Standing -> HOME."""
    asset = env.scene["robot"]
    if not hasattr(env, "_swz_ids"):
        n = asset.joint_names
        env._swz_ids = [n.index(j) for j in ("left_hip_roll", "right_hip_roll", "left_hip_yaw", "right_hip_yaw")]
        env._swz_sg = torch.tensor([1.0, -1.0, -1.0, 1.0], device=env.device)
    ph = env.command_manager.get_command(clock)
    v = env.command_manager.get_command(command_name)[:, 0]
    mov = (v.abs() > v_min).float()
    s, c = ph[:, 0], ph[:, 1]
    d = torch.stack([amp * s * mov, amp * s * mov, toe * torch.sign(v) * c * mov, toe * torch.sign(v) * c * mov], -1) * env._swz_sg
    return env._swz_ids, asset.data.default_joint_pos[:, env._swz_ids] + d


def swizzle_gait_track(env: ManagerBasedRlEnv, clock: str = "swizzle_phase", command_name: str = "twist", amp: float = 0.15,
                       toe: float = 0.1745, std: float = 0.1) -> torch.Tensor:
    """exp(-mean((q - q_ref)^2) / std^2) over hip roll and hip yaw of both legs against swizzle_reference."""
    ids, ref = swizzle_reference(env, clock, command_name, amp, toe)
    q = env.scene["robot"].data.joint_pos[:, ids]
    return torch.exp(-((q - ref) ** 2).mean(-1) / std ** 2)


def _stride_parts(env, command_name, sensor_name, transition):
    asset = env.scene["robot"]
    sn = env.command_manager.get_command(command_name)[:, 0]
    on = _foot_states(env, sensor_name)
    if not hasattr(env, "_os_ank"):
        env._os_ank = [asset.find_bodies(b)[0][0] for b in ("ankle_l_v1", "ankle_r_v1")]
        names = asset.joint_names
        env._os_legs = [[i for i, n in enumerate(names) if n.startswith(pre) and any(k in n for k in ("hip", "knee", "ankle"))] for pre in ("left_", "right_")]
    window = sn.abs() < math.sin(math.pi * transition)                   # around the phase change: both skates down (the push)
    left_free = sn > 0
    return asset, on, left_free, window


def stride_reward(env: ManagerBasedRlEnv, command_name: str = "stride_phase", sensor_name: str = "feet_ground_contact",
                  lift_min: float = 0.02, lift_full: float = 0.04, swing_quiet_std: float = 0.3, transition: float = 0.15) -> torch.Tensor:
    """The skating stride, paid by the clock: outside the transition window the FREE skate (by phase) hangs still lift_min..
    lift_full above the rolling stance skate while the trunk rolls (v > 0.05); inside the window both skates are down (the
    set-down and push — the speed terms pay the push itself). 0..1 per step."""
    asset, on, left_free, window = _stride_parts(env, command_name, sensor_name, transition)
    z = asset.data.body_link_pos_w[:, env._os_ank, 2]
    lift = torch.where(left_free, z[:, 0] - z[:, 1], z[:, 1] - z[:, 0])
    lift_frac = ((lift - lift_min) / (lift_full - lift_min)).clamp(0.0, 1.0)
    vf = asset.data.body_link_lin_vel_w[:, env._os_ank] - asset.data.root_link_lin_vel_w.unsqueeze(1)
    vsw = torch.where(left_free, vf[:, 0].norm(dim=-1), vf[:, 1].norm(dim=-1))
    lift_frac = lift_frac * torch.exp(-(vsw / swing_quiet_std) ** 2)
    free_off = torch.where(left_free, ~on[:, 0], ~on[:, 1]); stance_on = torch.where(left_free, on[:, 1], on[:, 0])
    rolling = (_fwd_speed(asset) > 0.05).float()
    glide = lift_frac * free_off.float() * stance_on.float() * rolling
    both = on.all(dim=-1).float() * rolling
    return torch.where(window, both, glide)


def stride_wrong_foot(env: ManagerBasedRlEnv, command_name: str = "stride_phase", sensor_name: str = "feet_ground_contact",
                      transition: float = 0.15) -> torch.Tensor:
    """1 when, outside the transition window, the STANCE skate (by phase) is off the floor — the wrong foot lifted or both in
    the air (a hop). Penalty."""
    asset, on, left_free, window = _stride_parts(env, command_name, sensor_name, transition)
    stance_off = torch.where(left_free, ~on[:, 1], ~on[:, 0])
    return (stance_off & ~window).float()


# ---- Slope balance --------------------------------------------------------------------------------------------------

def _robot_com_xy(env, asset):
    """Mass-weighted centre of mass of the whole robot (world xy) and the support centre (mean of the tire bodies' xy)."""
    if not hasattr(env, "_com_m"):
        mjm = env.sim.mj_model
        names = list(asset.body_names)
        m = []
        for n in names:
            try:
                m.append(float(mjm.body("robot/" + n).mass[0]))
            except KeyError:
                m.append(float(mjm.body(n).mass[0]))
        env._com_m = torch.tensor(m, device=env.device).view(1, -1, 1)
        env._com_tires = [i for i, n in enumerate(names) if "tire" in n.lower()]
        assert len(env._com_tires) == 4, names
    com = (asset.data.body_com_pos_w * env._com_m).sum(1) / env._com_m.sum()
    sup = asset.data.body_com_pos_w[:, env._com_tires, :2].mean(1)
    return com[:, :2], sup


def com_fore_aft_offset(env: ManagerBasedRlEnv) -> torch.Tensor:
    """Signed fore-aft offset (m, + = ahead) of the vertical through the robot's CoM from the centre of the four wheel
    contacts, along the trunk heading. The static stability margin on a slope is half the skate wheelbase (32 mm)."""
    asset = env.scene["robot"]
    com, sup = _robot_com_xy(env, asset)
    w, x, y, z = asset.data.root_link_quat_w.unbind(-1)
    yaw = torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    d = com - sup
    return d[:, 0] * torch.cos(yaw) + d[:, 1] * torch.sin(yaw)


def com_over_support(env: ManagerBasedRlEnv, std: float = 0.015, d0: float = 0.0, tail: float = 0.0) -> torch.Tensor:
    """exp(-((offset - d0)/std)^2): 1 when the CoM's vertical is over the middle of the wheelbase (d0 = the HOME stance offset
    on the flat). tail > 0: a linear penalty -tail * (|x| - std)/std beyond one std (capped at 4 std), so the term keeps a
    gradient far from d0."""
    x = (com_fore_aft_offset(env) - d0) / std
    r = torch.exp(-x ** 2)
    if tail > 0:
        r = r - tail * (x.abs() - 1.0).clamp(min=0.0, max=4.0)
    return r


def single_glide(env: ManagerBasedRlEnv, sensor_name: str = "feet_ground_contact", lift_min: float = 0.005, lift_full: float = 0.04,
                 swing_quiet_std: float = 0.3) -> torch.Tensor:
    """Stepping stone for the stride: any skate hanging still lift_min..lift_full above the other while that one rolls
    (v > 0.05 m/s). No clock, no side."""
    asset = env.scene["robot"]
    on = _foot_states(env, sensor_name)
    if not hasattr(env, "_os_ank"):
        env._os_ank = [asset.find_bodies(b)[0][0] for b in ("ankle_l_v1", "ankle_r_v1")]
    z = asset.data.body_link_pos_w[:, env._os_ank, 2]
    vf = asset.data.body_link_lin_vel_w[:, env._os_ank] - asset.data.root_link_lin_vel_w.unsqueeze(1)
    rolling = (_fwd_speed(asset) > 0.05).float()
    out = torch.zeros(z.shape[0], device=z.device)
    for free, stance in ((0, 1), (1, 0)):
        lift = ((z[:, free] - z[:, stance] - lift_min) / (lift_full - lift_min)).clamp(0.0, 1.0)
        quiet = torch.exp(-(vf[:, free].norm(dim=-1) / swing_quiet_std) ** 2)
        out = torch.maximum(out, lift * quiet * (~on[:, free] & on[:, stance]).float() * rolling)
    return out


# ---- Circle at speed ------------------------------------------------------------------------------------------------

class CircleCommand(RelativeHeadingVelocityCommand):
    """The ride's speed command (cmd[0] from the ranges) plus a yaw-rate target in cmd[2]: sign random, magnitude
    yaw_min..yaw_max rad/s, held for the episode. Circle radius R = v / omega."""

    def _resample_command(self, env_ids: torch.Tensor) -> None:
        super()._resample_command(env_ids)
        n = len(env_ids)
        lo, hi = (float(x) for x in os.environ.get("MICRODUCK_ROLLER_CIRCLE", "0.5,1.5").split(","))
        mag = lo + (hi - lo) * torch.rand(n, device=self.device)
        self.vel_command_b[env_ids, 2] = torch.where(torch.rand(n, device=self.device) < 0.5, -mag, mag)

    def _update_command(self) -> None:
        keep = self.vel_command_b[:, 2].clone()
        super()._update_command()
        self.vel_command_b[:, 2] = keep


class CircleCommandCfg(RelativeHeadingVelocityCommandCfg):
    def build(self, env: ManagerBasedRlEnv) -> "CircleCommand":
        return CircleCommand(self, env)


# ---- One-skate "spiral": stand on one skate, free leg stretched back, trunk leaned forward, coast with no pushes,
# at the end put the second skate down. ------------------------------------------------------------------------------

def one_skate_spiral(env: ManagerBasedRlEnv, command_name: str = "twist", sensor_name: str = "feet_ground_contact",
                     back_min: float = 0.5, back_full: float = 1.0, lift_min: float = 0.03, swing_quiet_std: float = 0.3,
                     lean_target: float = 0.15, lean_std: float = 0.15) -> torch.Tensor:
    """cmd[1] != 0: the free skate (by command) is off the floor, >= lift_min above the stance one and still, its hip pitched
    BACK back_min..back_full rad (paid continuously), knee straight, the stance skate on the floor and rolling, the trunk
    leaned forward lean_target (Gaussian, std lean_std); product of the parts, 0..1. cmd[1] == 0 (the end hold): both skates
    on the floor and rolling -> 1."""
    asset = env.scene["robot"]
    side = env.command_manager.get_command(command_name)[:, 1]
    on = _foot_states(env, sensor_name)
    if not hasattr(env, "_os_ank"):
        env._os_ank = [asset.find_bodies(b)[0][0] for b in ("ankle_l_v1", "ankle_r_v1")]
    if not hasattr(env, "_sp_j"):
        n = asset.joint_names
        env._sp_j = {k: n.index(k) for k in ("left_hip_pitch", "right_hip_pitch", "left_knee", "right_knee")}
        env._sp_home = {"left_hip_pitch": -0.371, "right_hip_pitch": 0.371}
    q = asset.data.joint_pos
    left = side > 0
    z = asset.data.body_link_pos_w[:, env._os_ank, 2]
    lift = torch.where(left, z[:, 0] - z[:, 1], z[:, 1] - z[:, 0])
    lift_ok = ((lift - lift_min) / 0.02).clamp(0.0, 1.0)
    vf = asset.data.body_link_lin_vel_w[:, env._os_ank] - asset.data.root_link_lin_vel_w.unsqueeze(1)
    vsw = torch.where(left, vf[:, 0].norm(dim=-1), vf[:, 1].norm(dim=-1))
    quiet = torch.exp(-(vsw / swing_quiet_std) ** 2)
    # hip pitched back: left +, right - (mirrored), measured from HOME
    back_l = q[:, env._sp_j["left_hip_pitch"]] - env._sp_home["left_hip_pitch"]
    back_r = -(q[:, env._sp_j["right_hip_pitch"]] - env._sp_home["right_hip_pitch"])
    back = torch.where(left, back_l, back_r)
    back_frac = ((back - back_min) / (back_full - back_min)).clamp(0.0, 1.0)
    knee = torch.where(left, q[:, env._sp_j["left_knee"]], q[:, env._sp_j["right_knee"]]).abs()
    knee_straight = torch.exp(-(knee / 0.5) ** 2)
    g = asset.data.projected_gravity_b                                     # trunk pitch forward: gravity's x-component in the trunk
    pitch = torch.atan2(g[:, 0], -g[:, 2])                                 # + = nose down
    lean = torch.exp(-((pitch - lean_target) / lean_std) ** 2)
    free_off = torch.where(left, ~on[:, 0], ~on[:, 1]); stance_on = torch.where(left, on[:, 1], on[:, 0])
    rolling = (_fwd_speed(asset).abs() > 0.05).float()
    # Sum of shaped parts: the one-skate glide term plus leg-back and lean, paid whenever the free skate is off the floor.
    glide = lift_ok * quiet * free_off.float() * stance_on.float() * rolling
    shape = free_off.float() * stance_on.float() * (0.5 * back_frac * knee_straight + 0.25 * lean)
    # Pose hint: Gaussian on the joint targets (free hip back 0.9 rad, free knee straight, stance hip roll 0.23 rad toward
    # the stance skate), no contact gates.
    hint_back = torch.exp(-((back - 0.9) / 0.35) ** 2) * knee_straight
    if not hasattr(env, "_sp_roll"):
        n = asset.joint_names; env._sp_roll = {"left": n.index("left_hip_roll"), "right": n.index("right_hip_roll")}
    roll_st = torch.where(left, q[:, env._sp_roll["right"]] - 0.0873, -(q[:, env._sp_roll["left"]] + 0.0873))   # + = trunk over the stance skate
    hint_roll = torch.exp(-((roll_st - 0.23) / 0.15) ** 2)
    pose = (glide + shape + 0.5 * hint_back + 0.25 * hint_roll) / 2.5
    both = on.all(dim=-1).float() * rolling
    return torch.where(side == 0, both, pose)


def no_push_penalty(env: ManagerBasedRlEnv) -> torch.Tensor:
    """Speed gain per step (m/s, potential like brake_progress with the other sign): a push costs what it adds."""
    asset = env.scene["robot"]
    v = _fwd_speed(asset).abs()
    if not hasattr(env, "_np_prev") or env._np_prev.shape[0] != v.shape[0]:
        env._np_prev = v.clone()
    dv = (v - env._np_prev).clamp(min=0.0)
    env._np_prev = v.clone()
    return torch.where(env.episode_length_buf <= 1, torch.zeros_like(dv), dv)


# ---- Compass / pivot: one skate planted on its rear wheel (front wheel up), the other rolls a circle around it. ----------

def _tire_geometry(env, asset):
    if not hasattr(env, "_pv_tires"):
        names = list(asset.body_names)
        env._pv_tires = {"L": [names.index("tire"), names.index("tire_2")], "R": [names.index("tire_3"), names.index("tire_4")]}
    return env._pv_tires


def pivot_reward(env: ManagerBasedRlEnv, command_name: str = "twist", sensor_name: str = "feet_ground_contact", lift_min: float = 0.008,
                 lift_full: float = 0.02, omega_full: float = 2.0, still_std: float = 0.08, wheel_radius: float = 0.015) -> torch.Tensor:
    """cmd[1] > 0: the LEFT skate is the pivot (< 0: the right). Parts (0..1 each, product): the pivot's front wheel lift_min..
    lift_full above the floor while its rear wheel stays down (paid continuously from 0); the free skate's both wheels on the
    floor; the trunk yaw rate |w| up to omega_full (paid continuously); the pivot's rear wheel not travelling (exp(-(v/still)^2)).
    'Front' = the wheel further along the trunk heading."""
    asset = env.scene["robot"]
    side = env.command_manager.get_command(command_name)[:, 1]
    T = _tire_geometry(env, asset)
    pos = asset.data.body_com_pos_w; vel = asset.data.body_com_lin_vel_w if hasattr(asset.data, "body_com_lin_vel_w") else asset.data.body_link_lin_vel_w
    w, x, y, z = asset.data.root_link_quat_w.unbind(-1)
    yaw = torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    hx, hy = torch.cos(yaw), torch.sin(yaw)
    floor = env.scene.env_origins[:, 2] if env.scene.terrain is not None else torch.zeros_like(yaw)
    out = torch.zeros_like(yaw)
    for key, is_side in (("L", side > 0), ("R", side < 0)):
        ids = T[key]; other = T["R" if key == "L" else "L"]
        px = pos[:, ids, 0] * hx.unsqueeze(1) + pos[:, ids, 1] * hy.unsqueeze(1)                 # along heading
        front = px.argmax(dim=1); rear = 1 - front
        z_front = pos[:, ids, 2].gather(1, front.unsqueeze(1)).squeeze(1)
        z_rear = pos[:, ids, 2].gather(1, rear.unsqueeze(1)).squeeze(1)
        h_front = z_front - wheel_radius - floor; h_rear = z_rear - wheel_radius - floor
        lift = ((h_front - lift_min) / (lift_full - lift_min)).clamp(0.0, 1.0)
        rear_down = torch.exp(-(h_rear.clamp(min=0.0) / 0.004) ** 2)
        free_down = torch.exp(-((pos[:, other, 2] - wheel_radius - floor.unsqueeze(1)).clamp(min=0.0) / 0.004).pow(2).sum(1))
        v_rear = vel[:, ids, :2].gather(1, rear.view(-1, 1, 1).expand(-1, 1, 2)).squeeze(1).norm(dim=-1)
        still = torch.exp(-(v_rear / still_std) ** 2)
        wz = asset.data.root_link_ang_vel_w[:, 2]
        # Potential on the accumulated yaw: pay the increase of |sum of dpsi| since the episode start (normalized by
        # omega_full*dt); a reversal pays back what it earned. Computed once per step for all envs (the loop over sides
        # only picks the plant).
        if key == "L":
            if not hasattr(env, "_pv_acc") or env._pv_acc.shape[0] != wz.shape[0]:
                env._pv_acc = torch.zeros_like(wz)
            dpsi = wz * env.step_dt
            acc_new = torch.where(env.episode_length_buf <= 1, torch.zeros_like(wz), env._pv_acc + dpsi)
            env._pv_gain = (acc_new.abs() - env._pv_acc.abs()) / (omega_full * env.step_dt)
            env._pv_acc = acc_new
        spin = env._pv_gain.clamp(-1.0, 1.0)
        # The planted skate must be turned across (toe-in, hip yaw 0.5..1.5 rad) so its wheel cannot roll and becomes
        # the pivot.
        if not hasattr(env, "_pv_yaw_j"):
            n = asset.joint_names; env._pv_yaw_j = {"L": n.index("left_hip_yaw"), "R": n.index("right_hip_yaw")}
        turn = ((asset.data.joint_pos[:, env._pv_yaw_j[key]].abs() - 0.5) / 1.0).clamp(0.0, 1.0)
        r = lift * rear_down * free_down * still * spin * turn
        out = torch.where(is_side, r, out)
    return out


def pivot_front_up(env: ManagerBasedRlEnv, command_name: str = "twist", wheel_radius: float = 0.015) -> torch.Tensor:
    """Diagnostic / shaping: the pivot skate's front-wheel height above the floor (m)."""
    asset = env.scene["robot"]
    side = env.command_manager.get_command(command_name)[:, 1]
    T = _tire_geometry(env, asset); pos = asset.data.body_com_pos_w
    w, x, y, z = asset.data.root_link_quat_w.unbind(-1); yaw = torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    hx, hy = torch.cos(yaw), torch.sin(yaw)
    floor = env.scene.env_origins[:, 2] if env.scene.terrain is not None else torch.zeros_like(yaw)
    out = torch.zeros_like(yaw)
    for key, is_side in (("L", side > 0), ("R", side < 0)):
        ids = T[key]; px = pos[:, ids, 0] * hx.unsqueeze(1) + pos[:, ids, 1] * hy.unsqueeze(1); front = px.argmax(dim=1)
        h = pos[:, ids, 2].gather(1, front.unsqueeze(1)).squeeze(1) - wheel_radius - floor
        out = torch.where(is_side, h.clamp(min=0.0), out)
    return out


def pivot_turn_shaping(env: ManagerBasedRlEnv, command_name: str = "twist") -> torch.Tensor:
    """Shaping: |hip yaw| of the planted skate (rad, capped 1.5) — the toe-in that pins the pivot wheel."""
    asset = env.scene["robot"]
    side = env.command_manager.get_command(command_name)[:, 1]
    if not hasattr(env, "_pv_yaw_j"):
        n = asset.joint_names; env._pv_yaw_j = {"L": n.index("left_hip_yaw"), "R": n.index("right_hip_yaw")}
    q = asset.data.joint_pos
    return torch.where(side > 0, q[:, env._pv_yaw_j["L"]].abs(), q[:, env._pv_yaw_j["R"]].abs()).clamp(max=1.5)


# ---- Slalom "Fish": feet together, both skates swivel under the trunk at every cone, the trunk faces down the cone
# line, the path snakes. Hip yaw turns the skate 1:1; the same sign on both hips turns both skates the same way
# (+ = right). Path amplitude v*sin(A)*T/pi. ---------------------------------------------------------------------------

def _skate_yaw_rel(env, asset):
    """Heading of each skate (rear wheel -> front wheel, front fixed at the first call) relative to the trunk yaw, rad, (N, 2) L, R."""
    T = _tire_geometry(env, asset)
    pos = asset.data.body_link_pos_w
    w, x, y, z = asset.data.root_link_quat_w.unbind(-1)
    yaw = torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    if not hasattr(env, "_fish_front"):
        hx, hy = torch.cos(yaw[0]), torch.sin(yaw[0])
        env._fish_front = {}
        for key in ("L", "R"):
            a, b = pos[0, T[key][0]], pos[0, T[key][1]]
            env._fish_front[key] = (T[key][1], T[key][0]) if ((b - a)[0] * hx + (b - a)[1] * hy) > 0 else (T[key][0], T[key][1])
    out = []
    for key in ("L", "R"):
        f, r = env._fish_front[key]
        v = pos[:, f, :2] - pos[:, r, :2]
        out.append(torch.atan2(torch.sin(torch.atan2(v[:, 1], v[:, 0]) - yaw), torch.cos(torch.atan2(v[:, 1], v[:, 0]) - yaw)))
    return torch.stack(out, dim=1)


def fish_reward(env: ManagerBasedRlEnv, command_name: str = "fish_phase", sensor_name: str = "feet_ground_contact", amp: float = 0.28,
                std: float = 0.1, min_speed: float = 0.1, trunk_std: float = 0.1, heading_command: str = "twist") -> torch.Tensor:
    """The mean heading of the two skates relative to the trunk follows amp*sin(phase) (+ = left), Gaussian std, AND the trunk
    holds the line (spawn heading, Gaussian trunk_std), so the skates turn in the world and the path snakes; both skates on
    the floor, rolling forward > min_speed. 0..1 per step."""
    asset = env.scene["robot"]
    target = amp * env.command_manager.get_command(command_name)[:, 0]
    psi = _skate_yaw_rel(env, asset).mean(dim=1)
    on = _foot_states(env, sensor_name)
    rolling = (_fwd_speed(asset) > min_speed).float()
    w, x, y, z = asset.data.root_link_quat_w.unbind(-1)
    yaw = torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    line = getattr(env.command_manager.get_term(heading_command), "_target_heading_w", None)
    hold = torch.ones_like(yaw) if line is None else torch.exp(-(torch.atan2(torch.sin(yaw - line), torch.cos(yaw - line)) / trunk_std) ** 2)
    return torch.exp(-((psi - target) / std) ** 2) * hold * on.all(dim=-1).float() * rolling


# ---- Heel-toe: scissors, the lead skate rolls on its rear (heel) wheel with the toe wheel up, the trail
# skate on its front (toe) wheel with the heel wheel up: riding on two wheels on a diagonal. ---------------------------

def _wheel_heights(env, asset, key, wheel_radius=0.015):
    """(front wheel height, rear wheel height) above the floor, m, for skate key L/R; front/rear along the trunk heading."""
    T = _tire_geometry(env, asset); ids = T[key]; pos = asset.data.body_link_pos_w
    w, x, y, z = asset.data.root_link_quat_w.unbind(-1); yaw = torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    px = pos[:, ids, 0] * torch.cos(yaw).unsqueeze(1) + pos[:, ids, 1] * torch.sin(yaw).unsqueeze(1)
    front = px.argmax(dim=1, keepdim=True); rear = 1 - front
    floor = env.scene.env_origins[:, 2] if env.scene.terrain is not None else torch.zeros_like(yaw)
    zz = pos[:, ids, 2]
    return zz.gather(1, front).squeeze(1) - wheel_radius - floor, zz.gather(1, rear).squeeze(1) - wheel_radius - floor


def _heeltoe_parts(env, command_name):
    asset = env.scene["robot"]
    side = env.command_manager.get_command(command_name)[:, 1]              # > 0: the LEFT skate leads
    lf, lr = _wheel_heights(env, asset, "L"); rf, rr = _wheel_heights(env, asset, "R")
    lead_lift = torch.where(side > 0, lf, rf); lead_down = torch.where(side > 0, lr, rr)      # lead: toe up, heel down
    trail_lift = torch.where(side > 0, rr, lr); trail_down = torch.where(side > 0, rf, lf)    # trail: heel up, toe down
    return asset, side, lead_lift, lead_down, trail_lift, trail_down


def heeltoe_reward(env: ManagerBasedRlEnv, command_name: str = "twist", lift_min: float = 0.008, lift_full: float = 0.02,
                   ahead_full: float = 0.02, min_speed: float = 0.15) -> torch.Tensor:
    """0.5 x lead part + 0.5 x trail part (each: its wheel lift_min..lift_full up, paid continuously, x the other wheel of the
    skate on the floor), x the scissors (lead ankle ahead of the trail one, 0..ahead_full, paid 0.5..1) x rolling > min_speed."""
    asset, side, ll, ld, tl, td = _heeltoe_parts(env, command_name)
    ramp = lambda h: ((h - lift_min) / (lift_full - lift_min)).clamp(0.0, 1.0)
    down = lambda h: torch.exp(-(h.clamp(min=0.0) / 0.004) ** 2)
    lead = ramp(ll) * down(ld); trail = ramp(tl) * down(td)
    if not hasattr(env, "_os_ank"):
        env._os_ank = [asset.find_bodies(b)[0][0] for b in ("ankle_l_v1", "ankle_r_v1")]
    w, x, y, z = asset.data.root_link_quat_w.unbind(-1); yaw = torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    a = asset.data.body_link_pos_w[:, env._os_ank, :2]
    along = a[..., 0] * torch.cos(yaw).unsqueeze(1) + a[..., 1] * torch.sin(yaw).unsqueeze(1)
    ahead = torch.where(side > 0, along[:, 0] - along[:, 1], along[:, 1] - along[:, 0])
    sc = 0.5 + 0.5 * (ahead / ahead_full).clamp(0.0, 1.0)
    rolling = (_fwd_speed(asset) > min_speed).float()
    return (0.5 * lead + 0.5 * trail) * sc * rolling


def heeltoe_lift(env: ManagerBasedRlEnv, command_name: str = "twist") -> torch.Tensor:
    """Shaping: lead toe-wheel height + trail heel-wheel height (m, each capped 2 cm) — so the lifts are found."""
    _, _, ll, _, tl, _ = _heeltoe_parts(env, command_name)
    return ll.clamp(0.0, 0.02) + tl.clamp(0.0, 0.02)


# ---- Slalom "Snake": Fish with the feet in scissors, one skate behind the other. Targets: lead ankle >= ahead_full
# ahead, skate axes <= across_max apart. cmd[1] = +1: the left skate leads. -------------------------------------------

def _snake_offsets(env, asset, command_name):
    side = env.command_manager.get_command(command_name)[:, 1]
    if not hasattr(env, "_os_ank"):
        env._os_ank = [asset.find_bodies(b)[0][0] for b in ("ankle_l_v1", "ankle_r_v1")]
    w, x, y, z = asset.data.root_link_quat_w.unbind(-1); yaw = torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    T = _tire_geometry(env, asset); pos = asset.data.body_link_pos_w
    cl = pos[:, T["L"], :2].mean(1); cr = pos[:, T["R"], :2].mean(1)                     # skate centres
    dv = cl - cr
    along = dv[:, 0] * torch.cos(yaw) + dv[:, 1] * torch.sin(yaw)
    across = (-dv[:, 0] * torch.sin(yaw) + dv[:, 1] * torch.cos(yaw)).abs()
    ahead = torch.where(side > 0, along, -along)
    return ahead, across


def snake_reward(env: ManagerBasedRlEnv, command_name: str = "twist", fish_command: str = "fish_phase", amp: float = 0.28,
                 std: float = 0.1, ahead_full: float = 0.04, across_max: float = 0.05) -> torch.Tensor:
    """fish_reward (skates swivel together amp*sin under the trunk that holds the line, both down, rolling) x (0.3 + 0.7 x
    scissors), scissors = lead skate ahead 0..ahead_full (continuous) x exp(-((across - across_max)+ / 2 cm)^2)."""
    asset = env.scene["robot"]
    ahead, across = _snake_offsets(env, asset, command_name)
    sc = (ahead / ahead_full).clamp(0.0, 1.0) * torch.exp(-((across - across_max).clamp(min=0.0) / 0.02) ** 2)
    return fish_reward(env, command_name=fish_command, amp=amp, std=std) * (0.3 + 0.7 * sc)


def heeltoe_excess(env: ManagerBasedRlEnv, command_name: str = "twist", cap: float = 0.025) -> torch.Tensor:
    """Lift above `cap` (m), both skates summed."""
    _, _, ll, _, tl, _ = _heeltoe_parts(env, command_name)
    return (ll - cap).clamp(min=0.0) + (tl - cap).clamp(min=0.0)


def trunk_pitch_sq(env: ManagerBasedRlEnv) -> torch.Tensor:
    """Trunk pitch^2 (rad^2) from the projected gravity."""
    g = env.scene["robot"].data.projected_gravity_b
    return torch.atan2(g[:, 0], -g[:, 2]) ** 2



def snake_scissors(env: ManagerBasedRlEnv, command_name: str = "twist", ahead_full: float = 0.05) -> torch.Tensor:
    """Snake shaping: the lead skate's lead over the other, m, 0..ahead_full, while both skates are down."""
    asset = env.scene["robot"]
    ahead, _ = _snake_offsets(env, asset, command_name)
    on = _foot_states(env)
    return ahead.clamp(0.0, ahead_full) * on.all(dim=-1).float()


def snake_spread(env: ManagerBasedRlEnv, command_name: str = "twist", across_max: float = 0.05) -> torch.Tensor:
    """Skate axes farther apart than across_max, m."""
    _, across = _snake_offsets(env, env.scene["robot"], command_name)
    return (across - across_max).clamp(min=0.0)



def reset_rolling_entry_frac(env: ManagerBasedRlEnv, env_ids: torch.Tensor | None, frac: float = 0.6,
                             speed_range: tuple = (0.1, 0.45)) -> None:
    """A random share `frac` of the reset envs starts rolling (microduck_mdp.reset_rolling_entry), the rest standing."""
    from mjlab_microduck.tasks import mdp as microduck_mdp
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device)
    pick = env_ids[torch.rand(len(env_ids), device=env.device) < frac]
    if len(pick):
        microduck_mdp.reset_rolling_entry(env, pick, speed_range=speed_range)



def head_band_excess(env: ManagerBasedRlEnv, bands_deg: tuple = (10.0, 10.0, 8.0)) -> torch.Tensor:
    """Head kept upright: squared excess of neck_pitch / head_pitch / head_roll beyond a band around HOME (rad^2, summed).
    Inside the band the head is free."""
    asset = env.scene["robot"]
    if not hasattr(env, "_hb_ids"):
        n = asset.joint_names
        env._hb_ids = [n.index(j) for j in ("neck_pitch", "head_pitch", "head_roll")]
    q = asset.data.joint_pos[:, env._hb_ids] - asset.data.default_joint_pos[:, env._hb_ids]
    band = torch.tensor([math.radians(b) for b in bands_deg], device=q.device)
    return ((q.abs() - band).clamp(min=0.0) ** 2).sum(dim=-1)


# ---- Crouch while riding (a command; Pollen's RollerCrouch is a one-shot trick) -------------------------------------
# cmd[1] = depth u in [0, 0.3] (0.3 = full depth; small so the pose term's sigma choice by |cmd| stays "walking"),
# drawn with the speed command, zero with zero_prob; the trunk height target = z_home - (u / 0.3) * depth.

class CrouchCommand(SpawnHeadingVelocityCommand):
    def _resample_command(self, env_ids: torch.Tensor) -> None:
        super()._resample_command(env_ids)
        n = len(env_ids)
        if n == 0:
            return
        u = 0.3 * torch.rand(n, device=self.device)
        _cr = os.environ.get("MICRODUCK_ROLLER_CROUCH", "0.04,4,0.3").split(",")   # zero_prob from the switch (no extra cfg field:
        zp = float(_cr[2]) if len(_cr) > 2 else 0.3                                   # the roller-standup factory re-wraps vars(cfg))
        self.vel_command_b[env_ids, 1] = torch.where(torch.rand(n, device=self.device) < zp, torch.zeros_like(u), u)

    def _update_command(self) -> None:
        keep = self.vel_command_b[:, 1].clone()
        super()._update_command()
        self.vel_command_b[:, 1] = keep


class CrouchCommandCfg(SpawnHeadingVelocityCommandCfg):
    def build(self, env: ManagerBasedRlEnv) -> "CrouchCommand":
        return CrouchCommand(self, env)


def crouch_height_track(env: ManagerBasedRlEnv, command_name: str = "twist", depth: float = 0.04, z_home: float = 0.135,
                        std: float = 0.01) -> torch.Tensor:
    """exp(-((z - z_target)/std)^2), z_target = z_home - (cmd[1]/0.3) * depth; z = trunk above the terrain origin."""
    asset = env.scene["robot"]
    u = (env.command_manager.get_command(command_name)[:, 1] / 0.3).clamp(0.0, 1.0)
    z = asset.data.root_link_pos_w[:, 2] - env.scene.env_origins[:, 2]
    return torch.nan_to_num(torch.exp(-((z - (z_home - u * depth)) / std) ** 2), nan=0.0)


# ---- Spin about a planted skate (MICRODUCK_SPIN_PIVOT): built on the spin (yaw-rate tracking + the scissor push).
# cmd[1] = which skate is the plant (+1 left, -1 right); the plant's front wheel is lifted 5-20 mm (pivot on the rear
# wheel), the plant's rear wheel may drift only slowly (|v| <= w*r_max), the free skate stays >= d_min from the plant's
# rear wheel. The plant angle is not prescribed.

class SpinPivotCommand(SpinCommand):
    def _resample_command(self, env_ids: torch.Tensor) -> None:
        super()._resample_command(env_ids)
        self.vel_command_b[env_ids, 1] = torch.where(torch.rand(len(env_ids), device=self.device) < 0.5, -1.0, 1.0)


class SpinPivotCommandCfg(RelativeHeadingVelocityCommandCfg):
    def build(self, env: ManagerBasedRlEnv) -> "SpinPivotCommand":
        return SpinPivotCommand(self, env)


def _plant_wheels(env, asset, command_name):
    """(rear wheel world xy pos, rear wheel xy vel, front height above floor) of the plant skate chosen by cmd[1]."""
    side = env.command_manager.get_command(command_name)[:, 1]
    T = _tire_geometry(env, asset); pos = asset.data.body_com_pos_w; vel = asset.data.body_com_lin_vel_w
    w, x, y, z = asset.data.root_link_quat_w.unbind(-1); yaw = torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    hx, hy = torch.cos(yaw), torch.sin(yaw)
    floor = env.scene.env_origins[:, 2] if env.scene.terrain is not None else torch.zeros_like(yaw)
    rp = torch.zeros(yaw.shape[0], 2, device=yaw.device); rv = torch.zeros_like(rp); fh = torch.zeros_like(yaw)
    for key, is_side in (("L", side > 0), ("R", side < 0)):
        ids = T[key]; px = pos[:, ids, 0] * hx.unsqueeze(1) + pos[:, ids, 1] * hy.unsqueeze(1)
        front = px.argmax(dim=1); rear = px.argmin(dim=1)
        rp_k = pos[:, ids, :2].gather(1, rear.view(-1, 1, 1).expand(-1, 1, 2)).squeeze(1)
        rv_k = vel[:, ids, :2].gather(1, rear.view(-1, 1, 1).expand(-1, 1, 2)).squeeze(1)
        fh_k = pos[:, ids, 2].gather(1, front.unsqueeze(1)).squeeze(1) - 0.015 - floor
        rp = torch.where(is_side.unsqueeze(1), rp_k, rp); rv = torch.where(is_side.unsqueeze(1), rv_k, rv); fh = torch.where(is_side, fh_k, fh)
    return side, rp, rv, fh


def pivot_lift_band(env: ManagerBasedRlEnv, command_name: str = "twist", h_min: float = 0.005, h_max: float = 0.02) -> torch.Tensor:
    """0..1: the plant's front wheel h_min..h_max above the floor (paid continuously from 0, 0 above h_max + 1 cm)."""
    asset = env.scene["robot"]
    _, _, _, fh = _plant_wheels(env, asset, command_name)
    up = (fh / h_min).clamp(0.0, 1.0)
    return torch.nan_to_num(up * (fh < h_max + 0.01).float(), nan=0.0)


def pivot_rear_drift(env: ManagerBasedRlEnv, command_name: str = "twist", r_max: float = 0.03) -> torch.Tensor:
    """(|v_rear| - |w_z| * r_max)+^2 [m^2/s^2]: the plant's rear wheel may travel only as a circle of radius r_max (>= 0 -> negative weight)."""
    asset = env.scene["robot"]
    _, _, rv, _ = _plant_wheels(env, asset, command_name)
    allow = asset.data.root_link_ang_vel_w[:, 2].abs() * r_max
    over = (rv.norm(dim=-1) - allow).clamp(min=0.0)
    return torch.nan_to_num(over * over, nan=0.0)


def pivot_spacing(env: ManagerBasedRlEnv, command_name: str = "twist", d_min: float = 0.10, d_lo: float = 0.06,
                  sensor_name: str = "feet_ground_contact") -> torch.Tensor:
    """0..1: the free skate (ankle body) d_lo..d_min from the plant's rear wheel in the floor plane, and on the floor."""
    asset = env.scene["robot"]
    side, rp, _, _ = _plant_wheels(env, asset, command_name)
    if not hasattr(env, "_os_ank"):
        env._os_ank = [asset.find_bodies(b)[0][0] for b in ("ankle_l_v1", "ankle_r_v1")]
    ank = asset.data.body_link_pos_w[:, env._os_ank, :2]
    free = torch.where((side > 0).unsqueeze(1), ank[:, 1], ank[:, 0])
    d = (free - rp).norm(dim=-1)
    on = _foot_states(env, sensor_name)
    free_on = torch.where(side > 0, on[:, 1], on[:, 0]).float()
    return torch.nan_to_num(((d - d_lo) / (d_min - d_lo)).clamp(0.0, 1.0) * free_on, nan=0.0)


# ---- Crouch posture: the pose reference follows the depth command, as Pollen's crouch_glide_pose_by_phase does. ---
# desired = HOME + (cmd[1]/0.3) * delta, delta = the crouch pose (left leg; right mirrored).
# The speed regime uses cmd[0] and cmd[2] only (cmd[1] is the depth, not a lateral speed).
from mjlab.tasks.velocity.mdp.rewards import variable_posture as _variable_posture


class crouch_posture(_variable_posture):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        asset = env.scene[cfg.params["asset_cfg"].name]
        ids, names = asset.find_joints(cfg.params["asset_cfg"].joint_names)
        dh, dk, da = cfg.params["crouch_delta"]
        d = []
        for n in names:
            s = 1.0 if n.startswith("left") else -1.0
            d.append(s * dh if n.endswith("hip_pitch") else s * dk if n.endswith("knee") else s * da if n.endswith("ankle") else 0.0)
        self.delta = torch.tensor(d, device=env.device, dtype=torch.float32)

    def __call__(self, env, std_standing, std_walking, std_running, asset_cfg, command_name, crouch_delta=None,
                 walking_threshold: float = 0.5, running_threshold: float = 1.5):
        asset = env.scene[asset_cfg.name]
        command = env.command_manager.get_command(command_name)
        u = (command[:, 1] / 0.3).clamp(0.0, 1.0)
        total_speed = command[:, 0].abs() + command[:, 2].abs()
        standing = (total_speed < walking_threshold).float().unsqueeze(1)
        walking = ((total_speed >= walking_threshold) & (total_speed < running_threshold)).float().unsqueeze(1)
        running = (total_speed >= running_threshold).float().unsqueeze(1)
        std = self.std_standing * standing + self.std_walking * walking + self.std_running * running
        desired = self.default_joint_pos[:, asset_cfg.joint_ids] + u.unsqueeze(1) * self.delta
        err = torch.square(asset.data.joint_pos[:, asset_cfg.joint_ids] - desired)
        return torch.exp(-torch.mean(err / (std ** 2), dim=1))


def crouch_posture_l1(env: ManagerBasedRlEnv, command_name: str = "twist", crouch_delta=(0.40, 1.20, 0.80)) -> torch.Tensor:
    """-mean|q - (HOME + (cmd[1]/0.3) * delta)| over the leg servos (self-negating -> positive weight). Pollen's crouch bootstrap
    (crouch_glide_pose_l1): a constant gradient toward the crouch pose where the Gaussian posture term is ~0."""
    asset = env.scene["robot"]
    if not hasattr(env, "_cp_ids"):
        names = list(asset.joint_names)
        ids, d = [], []
        dh, dk, da = crouch_delta
        for i, n in enumerate(names):
            if n.startswith("passive_") or not any(t in n for t in ("hip", "knee", "ankle")):
                continue
            s = 1.0 if n.startswith("left") else -1.0
            ids.append(i); d.append(s * dh if n.endswith("hip_pitch") else s * dk if n.endswith("knee") else s * da if n.endswith("ankle") else 0.0)
        env._cp_ids = ids; env._cp_d = torch.tensor(d, device=env.device)
    u = (env.command_manager.get_command(command_name)[:, 1] / 0.3).clamp(0.0, 1.0)
    q = asset.data.joint_pos[:, env._cp_ids]
    target = asset.data.default_joint_pos[:, env._cp_ids] + u.unsqueeze(1) * env._cp_d
    return torch.nan_to_num(-(q - target).abs().mean(dim=-1), nan=0.0)


def glide_posture_l1(env: ManagerBasedRlEnv, delta=(0.29, 1.20, 0.91)) -> torch.Tensor:
    """-mean|q - (HOME + delta)| over hip_pitch / knee / ankle (left +, right mirrored; hip yaw/roll free for the plough and steering).
    Self-negating -> positive weight."""
    asset = env.scene["robot"]
    if not hasattr(env, "_gp_ids"):
        ids, d = [], []
        for i, n in enumerate(asset.joint_names):
            if n.startswith("passive_") or not n.endswith(("hip_pitch", "knee", "ankle")):
                continue
            s = 1.0 if n.startswith("left") else -1.0
            ids.append(i); d.append(s * (delta[0] if n.endswith("hip_pitch") else delta[1] if n.endswith("knee") else delta[2]))
        env._gp_ids = ids; env._gp_d = torch.tensor(d, device=env.device)
    q = asset.data.joint_pos[:, env._gp_ids]
    return torch.nan_to_num(-(q - asset.data.default_joint_pos[:, env._gp_ids] - env._gp_d).abs().mean(dim=-1), nan=0.0)


def reset_glide_pose(env: ManagerBasedRlEnv, env_ids, delta=(0.29, 1.20, 0.91), prob: float = 0.5, stand_z: float = 0.105,
                     lift: float = 0.005, spawn_on_ramp: float = 0.3, yaw: float = 0.0) -> None:
    """Reference-state start in the descent pose: a `prob` share of resets starts in the pose (hip_pitch/knee/ankle at
    HOME + delta, right leg mirrored), trunk stand_z above the surface, tilted to the ramp on the on-ramp column as
    reset_on_ramp_pose does. Runs after reset_robot_joints, before the rolling entry."""
    asset = env.scene["robot"]
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device)
    ids = env_ids[torch.rand(len(env_ids), device=env.device) < prob]
    if len(ids) == 0:
        return
    if not hasattr(env, "_rgp_ids"):
        jid, d = [], []
        for i, n in enumerate(asset.joint_names):
            if n.startswith("passive_") or not n.endswith(("hip_pitch", "knee", "ankle")):
                continue
            s = 1.0 if n.startswith("left") else -1.0
            jid.append(i); d.append(s * (delta[0] if n.endswith("hip_pitch") else delta[1] if n.endswith("knee") else delta[2]))
        env._rgp_ids = torch.tensor(jid, device=env.device); env._rgp_d = torch.tensor(d, device=env.device)
    q = asset.data.default_joint_pos[ids][:, env._rgp_ids] + env._rgp_d
    asset.write_joint_state_to_sim(q, torch.zeros_like(q), joint_ids=env._rgp_ids, env_ids=ids)
    oz = env.scene.env_origins[ids, 2]
    a = torch.where(oz < -1e-4, torch.atan2(-oz, torch.full_like(oz, spawn_on_ramp)), torch.zeros_like(oz))
    pos = env.scene.env_origins[ids].clone()
    pos[:, 2] = pos[:, 2] + stand_z * torch.cos(a) + lift
    hy = torch.full_like(a, yaw / 2.0); h = a / 2.0
    w1, x1, y1, z1 = torch.cos(h), torch.zeros_like(h), torch.sin(h), torch.zeros_like(h)
    w2, x2, y2, z2 = torch.cos(hy), torch.zeros_like(hy), torch.zeros_like(hy), torch.sin(hy)
    qn = torch.stack([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                      w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2], -1)
    asset.write_root_link_pose_to_sim(torch.cat([pos, qn], -1), env_ids=ids)


def crouch_gated_speed_track(env: ManagerBasedRlEnv, command_name: str = "twist", std: float = 0.25, wheel_radius: float = 0.015,
                             sensor_name: str = "feet_ground_contact", slip_std: float = 0.1) -> torch.Tensor:
    """rolling_speed_track x (1 - cmd[1]/0.3): in the crouch the duck coasts."""
    u = (env.command_manager.get_command(command_name)[:, 1] / 0.3).clamp(0.0, 1.0)
    return rolling_speed_track(env, command_name, std, wheel_radius, sensor_name, slip_std) * (1.0 - u)


def crouch_gated_quality_track(env: ManagerBasedRlEnv, command_name: str = "twist", std: float = 0.15, lat_std: float = 0.1,
                               sensor_name: str = "feet_ground_contact") -> torch.Tensor:
    """rolling_quality_track x (1 - cmd[1]/0.3): the same coast-in-the-crouch gate as crouch_gated_speed_track, for the
    rolling_quality_track credit (takes lat_std)."""
    u = (env.command_manager.get_command(command_name)[:, 1] / 0.3).clamp(0.0, 1.0)
    return rolling_quality_track(env, command_name, std, lat_std, sensor_name) * (1.0 - u)


def crouch_gated_swizzle_gait(env: ManagerBasedRlEnv, clock: str = "swizzle_phase", command_name: str = "twist", amp: float = 0.15,
                              toe: float = 0.1745, std: float = 0.1) -> torch.Tensor:
    """swizzle_gait_track x (1 - cmd[1]/0.3): the push credit fades with the crouch depth u."""
    u = (env.command_manager.get_command(command_name)[:, 1] / 0.3).clamp(0.0, 1.0)
    return swizzle_gait_track(env, clock, command_name, amp, toe, std) * (1.0 - u)


def hip_roll_stance(env: ManagerBasedRlEnv, target: float = 0.204, std: float = 0.1) -> torch.Tensor:
    """exp(-((roll_out - target)/std)^2) averaged over both hips, roll_out = outward hip roll (left +, right mirrored), absolute rad."""
    asset = env.scene["robot"]
    if not hasattr(env, "_hrs_ids"):
        n = list(asset.joint_names); env._hrs_ids = (n.index("left_hip_roll"), n.index("right_hip_roll"))
    q = asset.data.joint_pos
    out = torch.stack([q[:, env._hrs_ids[0]], -q[:, env._hrs_ids[1]]], dim=1)
    return torch.nan_to_num(torch.exp(-((out - target) / std) ** 2).mean(dim=1), nan=0.0)
