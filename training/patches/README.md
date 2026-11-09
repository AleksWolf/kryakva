# Patches to microduck_rl

Training is done in Pollen's `microduck_rl` (mjlab, MuJoCo Warp). The patch and the new files adapt it to this duck:
the model from [training/model](../model), the HD-1910 servo model, the FallSafe and roller stand-up tasks.
The other Pollen tasks are unchanged.

## Installation

```
git clone https://github.com/pollen-robotics/microduck_rl
cd microduck_rl && git checkout cb70b79
git apply <kryakva>/training/patches/microduck_rl_hd1910.patch
cp -r <kryakva>/training/patches/files/src .
```

Then install as described in the `microduck_rl` README. After that, patch MuJoCo Warp once:

```
python <kryakva>/training/patches/patch_mujoco_warp.py
```

The script fixes the contact computation between the floor and the round roller tire. Without the fix the tire touches the floor with a 10 mm patch along the travel, and the wheel
brakes in simulation like a flat pad. Soles and trunk are computed as in the original engine.
Without the fix no task runs: training stops with an error and asks you to run the script.
Running it again is safe; after updating `mujoco_warp`, run it again. To restore the original engine — the `--revert` flag.

## Environment variables

Common to all three policies:

```
MICRODUCK_ROBOT=hd1910
MICRODUCK_EXPORT_DIR=<kryakva>/training/model
MICRODUCK_ACT_JSON=<kryakva>/training/actuators/hd1910_bam_bench1019_D0.json
MICRODUCK_ACT_KP=5  MICRODUCK_HOME_LEAN_DEG=5
MICRODUCK_ACTION_SCALE=1.0  MICRODUCK_ACTION_CLIP=limits  MICRODUCK_ACTION_FILTER_TAU=0.00857  MICRODUCK_BACKLASH_GAP_DEG=0.5
```

Run: `train <task> --env.scene.num-envs <N> --agent.max-iterations <I> --agent.logger tensorboard --agent.save-interval 250`.

| Policy | Task | Additional | Envs | Iterations |
|---|---|---|---|---|
| walking | `Mjlab-Velocity-Flat-MicroDuck` | `MICRODUCK_WALK_LIMIT_MARGIN=0.08,2 MICRODUCK_CAM_STAB=0.35,1.5,0.15` | 4096 | 7992 |
| standing up | `Mjlab-StandUp-Flat-MicroDuck` | `MICRODUCK_STANDUP_ALLCOLL=1 MICRODUCK_ALLCOLL_MODEL=ext MICRODUCK_STANDUP_FALL_BANK=<kryakva>/training/data/standup_fall_bank_P5_1021_all.npz,0.3 MICRODUCK_STANDUP_LIMIT_MARGIN=0.08,2 MICRODUCK_STANDUP_RISE_CAP=0.15,5 MICRODUCK_SIM_BUFFERS=85,370` | 2048 | 2998 |
| FallSafe | `Mjlab-FallSafeHD-Flat-MicroDuck` | `MICRODUCK_ALLCOLL_MODEL=ext MICRODUCK_EXPERT_BC=0 MICRODUCK_ENTROPY_COEF=0.003 MICRODUCK_FALLSAFE_BANK=<kryakva>/training/data/fallsafe_bank_pred010_35_walkP5_1021.npz MICRODUCK_SIM_BUFFERS=75,350` | 4096 | 1999 |

[training/data](../data) — banks of start states: poses after a fall (standing up), states at the moment the protection triggers (FallSafe). `MICRODUCK_SIM_BUFFERS` — the number of contacts and constraints per env; for another model it has to be tuned again.
