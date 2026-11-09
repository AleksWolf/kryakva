"""Protective fall (FallSafe) env for the HD-1910 duck.

Built from Pollen's VelStand env parts, not its recipe: the walk env (obs 61D, DR, delays, HOME, action scale
from the MICRODUCK_* switches) + the external-collision model + VelStand's floor-impact sensors (servo housings named
at load, head subtree, trunk). Every walking/recovery reward, curriculum, push and prone spawn is removed.

Episode: FALLSAFE_EPISODE_S from a hand-over state (the walk's state at the moment its trunk tilt crosses theta_on
after a push); MICRODUCK_FALLSAFE_BANK="<npz>". Commands are tiny noise around zero (the runtime feeds zeros).

Rewards (tasks/fallsafe_hd1910.py and the mdp costs):
  end_upright      standing in the last 0.5 s and never tilted past 45°
  end_rest         lying still in the last 0.5 s
  head/servo/trunk_impact  floor force above 16 / 2 / 22 N
  servo_acc_spike  joint |accel| above 240 rad/s^2
  servo_motor_line load above 90 % of the HD-1910 motor line
  joint_limit_margin  joints within 0.1 rad of their stops
  action_rate / dof_pos_limits / self_collisions  small regularisers
Weights: MICRODUCK_FALLSAFE_W (see below).
"""

import os

from mjlab.managers import EventTermCfg, RewardTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.envs import mdp

from mjlab_microduck.tasks import fallsafe_hd1910 as fs
from mjlab_microduck.tasks import mdp as microduck_mdp
from mjlab_microduck.tasks.microduck_standup_env_cfg import MicroduckStandUpRlCfg
from mjlab_microduck.tasks.microduck_velstand_env_cfg import make_microduck_velstand_env_cfg
from mjlab_microduck.tasks.symmetry import SYMMETRY_CFG

FALLSAFE_EPISODE_S = 2.0
_SERVOS = SceneEntityCfg("robot", joint_names=(r"^(?!passive_).*",))
_KEEP_REWARDS = ("dof_pos_limits", "self_collisions")
_KEEP_TERMS = ("time_out", "nan_state", "out_of_terrain_bounds")
_DROP_EVENTS = ("push_robot", "topple_push", "random_prone_init")

# Default weights (per second of the condition / per unit above threshold per second); overridden by
# MICRODUCK_FALLSAFE_W="name=w,name=w,...".
W = {
    "end_upright": 4.0,
    "end_rest": 2.0,
    "head_impact": -2.0,
    "servo_impact": -5.0,
    "trunk_impact": -1.0,
    "servo_acc_spike": -1e-3,
    "servo_motor_line": -20.0,
    "joint_limit_margin": -15.0,
    "action_rate_l2": -0.05,
}


def make_microduck_fallsafe_hd1910_env_cfg(play: bool = False):
    cfg = make_microduck_velstand_env_cfg(play=play)
    cfg.episode_length_s = FALLSAFE_EPISODE_S
    for n in [n for n in cfg.rewards if n not in _KEEP_REWARDS]:
        cfg.rewards.pop(n)
    cfg.curriculum.clear()
    for n in _DROP_EVENTS:
        cfg.events.pop(n, None)
    for n in [n for n in cfg.terminations if n not in _KEEP_TERMS]:
        cfg.terminations.pop(n)

    tw = cfg.commands["twist"]
    tw.rel_standing_envs = 0.0
    tw.rel_heading_envs = 0.0
    tw.heading_command = False
    tw.ranges.heading = None
    tw.ranges.lin_vel_x = (-0.01, 0.01)
    tw.ranges.lin_vel_y = (-0.01, 0.01)
    tw.ranges.ang_vel_z = (-0.05, 0.05)
    tw.resampling_time_range = (FALLSAFE_EPISODE_S, 2 * FALLSAFE_EPISODE_S)

    w = dict(W)
    for kv in filter(None, os.environ.get("MICRODUCK_FALLSAFE_W", "").split(",")):
        k, v = kv.split("=")
        w[k.strip()] = float(v)
    from mjlab_microduck.actuator.rewards_hd1910 import servo_motor_line_excess
    cfg.rewards["end_upright"] = RewardTermCfg(func=fs.fs_end_upright, weight=w["end_upright"])
    cfg.rewards["end_rest"] = RewardTermCfg(func=fs.fs_end_rest, weight=w["end_rest"], params={"asset_cfg": _SERVOS})
    cfg.rewards["head_impact"] = RewardTermCfg(func=microduck_mdp.body_impact_cost, weight=w["head_impact"],
                                               params={"sensor_name": "head_ground_contact", "threshold": 16.0})
    cfg.rewards["servo_impact"] = RewardTermCfg(func=microduck_mdp.body_impact_cost, weight=w["servo_impact"],
                                                params={"sensor_name": "servo_ground_contact", "threshold": 2.0})
    cfg.rewards["trunk_impact"] = RewardTermCfg(func=microduck_mdp.body_impact_cost, weight=w["trunk_impact"],
                                                params={"sensor_name": "trunk_ground_contact", "threshold": 22.0})
    cfg.rewards["servo_acc_spike"] = RewardTermCfg(func=microduck_mdp.servo_acc_spike_penalty, weight=w["servo_acc_spike"],
                                                   params={"acc_thresh": 240.0})
    cfg.rewards["servo_motor_line"] = RewardTermCfg(func=servo_motor_line_excess, weight=w["servo_motor_line"], params={"frac": 0.9})
    cfg.rewards["joint_limit_margin"] = RewardTermCfg(func=microduck_mdp.joint_pos_limit_proximity, weight=w["joint_limit_margin"],
                                                      params={"margin": 0.1, "asset_cfg": _SERVOS})
    cfg.rewards["action_rate_l2"] = RewardTermCfg(func=mdp.action_rate_l2, weight=w["action_rate_l2"])

    # Hand-over states. Added last: reset events run in insertion order, so it overrides the base pose reset.
    bank = os.environ.get("MICRODUCK_FALLSAFE_BANK")
    if bank:
        from mjlab_microduck.actuator.events_hd1910 import reset_from_fall_bank
        cfg.events["reset_from_fall_bank"] = EventTermCfg(func=reset_from_fall_bank, mode="reset", params={"path": bank, "prob": 1.0})
        print(f"[microduck_fallsafe_hd1910_env_cfg] fall bank {bank}")
    else:   # the task registry builds every cfg at import; without a bank episodes start standing
        print("[microduck_fallsafe_hd1910_env_cfg] WARNING: MICRODUCK_FALLSAFE_BANK not set — episodes start standing")
    _sb = os.environ.get("MICRODUCK_SIM_BUFFERS")
    if _sb:
        cfg.sim.nconmax, cfg.sim.njmax = (int(x) for x in _sb.split(","))
        print(f"[microduck_fallsafe_hd1910_env_cfg] MICRODUCK_SIM_BUFFERS: nconmax {cfg.sim.nconmax}, njmax {cfg.sim.njmax}")
    print(f"[microduck_fallsafe_hd1910_env_cfg] weights {w}")
    return cfg


from copy import deepcopy  # noqa: E402

# StandUp's runner (plain PPO + optional mirror loss), not VelStand's (its BC targets Pollen's walk/stand experts).
# Symmetry on: the bank is recorded with pushes in random directions.
MicroduckFallSafeHd1910RlCfg = deepcopy(MicroduckStandUpRlCfg)
MicroduckFallSafeHd1910RlCfg.algorithm.symmetry_cfg = SYMMETRY_CFG
MicroduckFallSafeHd1910RlCfg.experiment_name = "microduck_fallsafe_hd1910"
MicroduckFallSafeHd1910RlCfg.run_name = "microduck_fallsafe_hd1910"
MicroduckFallSafeHd1910RlCfg.max_iterations = 2000
# Entropy coefficient 0.003 (action scale 0.3).
MicroduckFallSafeHd1910RlCfg.algorithm.entropy_coef = float(os.environ.get("MICRODUCK_ENTROPY_COEF", "0.003"))
