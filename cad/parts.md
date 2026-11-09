# Printed parts

All parts are in two assemblies: [cad/assembly.step](assembly.step) (the assembled duck) and [cad/assembly_rollers.step](assembly_rollers.step) (on rollers).
Right parts are mirror copies of the left ones. Quantities in the table are per duck; the same ones are in the profiles [print/](../print/README.md).

## Parts

| Part | Qty | Material |
|---|---|---|
| `trunk_base` | 1 | PLA |
| `left_shell`, `right_shell` | 1 + 1 | PLA |
| `power_support` | 1 | PLA |
| `neck` | 2 | PLA |
| `hip` | 2 | PLA |
| `yaw2roll` | 2 | PLA |
| `bearing_roll` | 2 | PLA |
| `upper_leg_left`, `upper_leg_right` | 1 + 1 | PLA |
| `leg` | 2 | PLA |
| `rigidity_plate` | 2 | PLA |
| `ankle_left`, `ankle_right` | 1 + 1 | PLA |
| `ankle_axle` | 2 | PLA |
| `foot_left`, `foot_right` | 1 + 1 | PLA |
| `sole_left`, `sole_right` | 1 + 1 | TPU |
| `neck_pitch` | 1 | PLA |
| `bearing_neck` | 1 | PLA |
| `yaw_roll_motion` | 1 | PLA |
| `motor_support` | 1 | PLA |
| `face_part` | 1 | PLA |
| `top_head_shell` | 1 | PLA |
| `bottom_head_shell` | 1 | PLA |
| `noenoeil` | 1 | PLA |
| `jaw` | 1 | PLA |
| `jaw_soft` | 1 | TPU |
| `soft_mouth_top` | 1 | TPU |
| `zero clamp` | 1 | ASA |
| `roller` | 2 | PLA |
| `bushing` | 4 | PLA |
| `rim` | 4 | PLA |
| `tire` | 4 | TPU |

## Heat-set inserts

| Insert | Qty | Where |
|---|---|---|
| M2 × 4 × Ø3 | 13 | feet 2 (toe), `top_head_shell` 5 (3 bosses + 2 UC-01 standoffs), `face_part` 4 (Zero 3W standoffs), rollers 2 (toes) |
| M2 × 3 × Ø3.2 | 9 | `face_part` 6 (camera 4, ToF 2), `motor_support` 2 (amplifier), `right_shell` 1 (joint of the halves) |

Melt the inserts in before assembly, flush with the face.

## Bearings

| Bearing | Qty | Where |
|---|---|---|
| MR2216-2RS, 16 × 22 × 4 | 11 | hip yaw and roll, hip pitch, knee (1 per leg each), neck pitch, head yaw and roll |
| 6700 open, 10 × 15 × 3 | 3 | ankles 2, beak 1 |
| MR126ZZ, 6 × 12 × 4 | 8 | roller wheels, 2 per wheel |

## Fasteners

M2 screws are hex socket unless stated otherwise. The layout by joint is in the assembly guide ([docs/assembly](../docs/assembly/index.html)).

| Fastener | Qty |
|---|---|
| M2 × 4 | 32 |
| M2 × 6 | 87 |
| M2 × 8 | 7 |
| M2 × 12 | 4 |
| M2.5 × 6 | 4 |
| M2.5 × 6 countersunk | 2 (IMU) |
| M2 × 6 countersunk | 2 (foot toes) |
| M2.5 × 14 | 4 (wheel axles) |
| self-tapping screw M2.3 × 4 | 4 (upper_leg → rear servo bosses) |
| self-tapping screw M2.3 × 6 | 2 (ankle axle → rear servo boss) |
| self-tapping screw Ø1.36 × 3.3 | 2 (battery contacts, from the charger kit) |
| double-sided adhesive tape on paper backing | soft beak inserts |
| nylon cable tie, the shortest | 2 (20 cm cable to the edge of the hole above the knee servo) |

Where there is no heat-set insert, the M2 screw cuts its thread directly in the plastic. A screw goes no deeper than 2.5 mm into the servo horn thread: at 3 mm it hits the servo body.
