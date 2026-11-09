# robotd

Программа робота Pollen Robotics (https://github.com/pollen-robotics/microduck, Apache-2.0) с нашими патчами: сервы Feetech HD-1910,
IMU на I2C платы Zero, журнал такта, работа утки без ПК. Собрана под aarch64 (Radxa Zero 3W, glibc 2.31 и новее).

| Путь | Что |
|---|---|
| `bin/robotd`, `bin/robotctl`, `bin/padd` | программа робота, её командная строка, программа пульта |
| `robotd.toml` | настройки утки: шина серв, знаки суставов, установка IMU, политики, детектор подъёма |
| `policies/` | три политики P5 (ходьба, подъём, защитное падение) с их `.runtime.json` (те же, что в `training/policies/`) |
| `models/pickup_detector.onnx` | детектор подъёма Pollen (`models/LICENSE-microduck`) |
| `hear/libonnxruntime.so` | ONNX Runtime 1.28.2 для aarch64 (`hear/LICENSE-onnxruntime`, MIT) |
| `standalone/` | `install.sh`, `robotd.service`, `padd.service`, журнал systemd в ОЗУ |
| `src/` | патчи `0001…0023` к коммиту `95a7e91` microduck Pollen и `scripts/build_microduck.sh` |

Установка — [настройка Zero](../zero_setup.ru.md), раздел 4.

Сборка из исходников: рядом с `src/` склонировать pollen-robotics/microduck в папку `microduck`, затем из `src/`:

```
TARGET=aarch64 scripts/build_microduck.sh
```

Нужен Rust с `cargo zigbuild`. Скрипт собирает robotd, robotctl и tofd в `../microduck-duck/target/aarch64-unknown-linux-gnu/release/`;
padd собирается из того же дерева с патчами (крейт `padd/`).
