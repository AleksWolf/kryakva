"""Patch MuJoCo Warp: contact points of the roller tyre on the floor.

mujoco_warp/_src/collision_primitive.py, plane_convex: the extra contact points (b, c, d) are taken from hull vertices
within 1 mm of the lowest one; on the rounded Ø30 mm tyre that is a +-5 mm flat patch along the rolling direction.
This patch makes the tolerance per contact: condim 4 (only the roller tyres use it) -> 0.01 mm, all other contacts
(condim 1/3: soles, body) -> 1 mm as upstream. Safe to run again; re-run after updating mujoco_warp (importing
mjlab_microduck checks that the patch is present).
    python patch_mujoco_warp.py [--revert]"""
import sys
from pathlib import Path

import mujoco_warp

D = Path(mujoco_warp.__file__).parent / "_src"
EDITS = [
    (D / "collision_primitive.py",
     "def plane_convex(plane_normal: wp.vec3, plane_pos: wp.vec3, convex: Geom) -> Tuple[wp.vec4, mat43, wp.vec3]:",
     "def plane_convex(plane_normal: wp.vec3, plane_pos: wp.vec3, convex: Geom, tol: float) -> Tuple[wp.vec4, mat43, wp.vec3]:"),
    (D / "collision_primitive.py", "threshold = max_support - 1e-3", "threshold = max_support - tol"),
    (D / "collision_primitive.py", "threshold = wp.max(0.0, max_support - 1e-3)", "threshold = wp.max(0.0, max_support - tol)"),
    (D / "collision_primitive.py", "dist, pos, normal = plane_convex(plane.normal, plane.pos, convex)",
     "dist, pos, normal = plane_convex(plane.normal, plane.pos, convex, wp.where(condim == 4, 1e-5, 1e-3))  # microduck: tyre"),
    (D / "collision_driver_test.py", "dist, pos, normal = plane_convex(wp.vec3(0.0, 0.0, 1.0), wp.vec3(0.0), convex_in)",
     "dist, pos, normal = plane_convex(wp.vec3(0.0, 0.0, 1.0), wp.vec3(0.0), convex_in, 1e-3)"),
]
revert = "--revert" in sys.argv
for f, old, new in EDITS:
    s = f.read_text(encoding="utf-8")
    a, b = (new, old) if revert else (old, new)
    if s.count(b) == 1 and s.count(a) == 0:
        print(f"already {'reverted' if revert else 'patched'}: {f.name}: {b[:60]}"); continue
    assert s.count(a) == 1, f"{f.name}: expected 1 match of '{a[:70]}', found {s.count(a)}: mujoco_warp version changed, patch by hand"
    f.write_text(s.replace(a, b), encoding="utf-8"); print(f"{'reverted' if revert else 'patched'}: {f.name}: {b[:70]}")
print(f"mujoco_warp {getattr(mujoco_warp, '__version__', '?')}")
