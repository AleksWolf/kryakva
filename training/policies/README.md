# Policies

Three ONNX policies for the duck on HD-1910. Trained on the MJCF models from [training/model](../model) with the recipes from [training/patches](../patches/README.md).

| File | Skill | Command cmd[0..2] |
|---|---|---|
| [walk_hd1910_p5_v2_iter7992_home5.onnx](walk_hd1910_p5_v2_iter7992_home5.onnx) | walking | vx, vy, turn; head — 4 targets |
| [standup_hd1910_p5_v1_iter2998_home5.onnx](standup_hd1910_p5_v1_iter2998_home5.onnx) | standing up from the floor | zeros |
| [fallsafe_hd1910_p5_v1_iter1999_home5.onnx](fallsafe_hd1910_p5_v1_iter1999_home5.onnx) | FallSafe | zeros |

Walking and standing up work on the duck.

## Common to all

- Input 61: angular velocity, gravity, positions and velocities of 14 joints, previous action, command 3, head 4, trunk 6.
  Output 14 — joint order as in `names` of the file [walk_hd1910_p5_v2_reference_trace.npz](walk_hd1910_p5_v2_reference_trace.npz).
- Servos: RAM registers 50 = 5 (P), 51 = 0 (D) on all 14 leg and neck servos, verified by reading back; EEPROM is not touched.
- Target: `raw = clip(home + 1.0 · a, clip_lo, clip_hi)`; `home`, `clip_lo`, `clip_hi` — from the reference.
- Filter every 20 ms tick: `target = target_prev + 0.7 · (raw − target_prev)`. When the policy changes, the filter continues from the current target.

[walk_hd1910_p5_v2_reference_trace.npz](walk_hd1910_p5_v2_reference_trace.npz) — 500 walking ticks for numerical verification of the runtime: obs, a, target_raw, target, q, cmd,
names, home, alpha, clip_lo, clip_hi, scale, dt.

## Transitions

- Walking → FallSafe when tilt + 0.1 s × tilt rate > 35°. FallSafe 2 s with zero commands → StandUp if the duck is not standing.
  StandUp → walking when it has been standing for 0.5 s (tilt < 10°, trunk above 105 mm).
