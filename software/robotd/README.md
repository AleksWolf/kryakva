# robotd

Pollen Robotics' robot daemon (https://github.com/pollen-robotics/microduck, Apache-2.0) with our patches for Feetech HD-1910
servos, the IMU on the Zero's I2C, the tick journal and a duck without a PC. Built for aarch64 (Radxa Zero 3W, glibc 2.31+).

| Path | What |
|---|---|
| `bin/robotd`, `bin/robotctl`, `bin/padd` | the daemon, its command line, the pad daemon |
| `robotd.toml` | the duck's config: servo bus, signs, IMU mount, policies, pickup detector |
| `policies/` | the three P5 policies (walk, stand-up, FallSafe) with their `.runtime.json` (the same as `training/policies/`) |
| `models/pickup_detector.onnx` | Pollen's pickup detector (`models/LICENSE-microduck`) |
| `hear/libonnxruntime.so` | ONNX Runtime 1.28.2 for aarch64 (`hear/LICENSE-onnxruntime`, MIT) |
| `standalone/` | `install.sh`, `robotd.service`, `padd.service`, journald in RAM |
| `src/` | patches `0001…0023` against Pollen's microduck commit `95a7e91` and `scripts/build_microduck.sh` |

Install: [software/zero_setup.md](../zero_setup.md), section 4.

Build from source: clone pollen-robotics/microduck next to `src/` as `microduck`, then from `src/`:

```
TARGET=aarch64 scripts/build_microduck.sh
```

Needs Rust with `cargo zigbuild`. The script builds robotd, robotctl and tofd into
`../microduck-duck/target/aarch64-unknown-linux-gnu/release/`; padd is built from the same patched tree (crate `padd/`).
