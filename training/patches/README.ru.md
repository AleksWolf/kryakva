# Патчи к microduck_rl

Обучение — в `microduck_rl` Pollen (mjlab, MuJoCo Warp). Патч и новые файлы переводят его на эту утку:
модель из [training/model](../model), модель сервы HD-1910, задачи защитного падения и подъёма на роликах.
Остальные задачи Pollen не меняются.

## Установка

```
git clone https://github.com/pollen-robotics/microduck_rl
cd microduck_rl && git checkout cb70b79
git apply <kryakva>/training/patches/microduck_rl_hd1910.patch
cp -r <kryakva>/training/patches/files/src .
```

Дальше — установка по README `microduck_rl`. После неё один раз поправить MuJoCo Warp:

```
python <kryakva>/training/patches/patch_mujoco_warp.py
```

Скрипт правит расчёт касания пола круглой шиной ролика. Без правки шина касается пола пятном 10 мм вдоль хода, и колесо
в симуляции тормозит, как плоская площадка. Подошвы и корпус считаются как в исходном движке.
Без правки не запускается ни одна задача: обучение остановится с ошибкой и попросит запустить скрипт.
Повторный запуск безопасен; после обновления `mujoco_warp` — запустить снова. Вернуть исходный движок — ключ `--revert`.

## Переменные окружения

Общие для всех трёх политик:

```
MICRODUCK_ROBOT=hd1910
MICRODUCK_EXPORT_DIR=<kryakva>/training/model
MICRODUCK_ACT_JSON=<kryakva>/training/actuators/hd1910_bam_bench1019_D0.json
MICRODUCK_ACT_KP=5  MICRODUCK_HOME_LEAN_DEG=5
MICRODUCK_ACTION_SCALE=1.0  MICRODUCK_ACTION_CLIP=limits  MICRODUCK_ACTION_FILTER_TAU=0.00857  MICRODUCK_BACKLASH_GAP_DEG=0.5
```

Запуск: `train <задача> --env.scene.num-envs <N> --agent.max-iterations <I> --agent.logger tensorboard --agent.save-interval 250`.

| Политика | Задача | Дополнительно | Сред | Итераций |
|---|---|---|---|---|
| ходьба | `Mjlab-Velocity-Flat-MicroDuck` | `MICRODUCK_WALK_LIMIT_MARGIN=0.08,2 MICRODUCK_CAM_STAB=0.35,1.5,0.15` | 4096 | 7992 |
| подъём | `Mjlab-StandUp-Flat-MicroDuck` | `MICRODUCK_STANDUP_ALLCOLL=1 MICRODUCK_ALLCOLL_MODEL=ext MICRODUCK_STANDUP_FALL_BANK=<kryakva>/training/data/standup_fall_bank_P5_1021_all.npz,0.3 MICRODUCK_STANDUP_LIMIT_MARGIN=0.08,2 MICRODUCK_STANDUP_RISE_CAP=0.15,5 MICRODUCK_SIM_BUFFERS=85,370` | 2048 | 2998 |
| защитное падение | `Mjlab-FallSafeHD-Flat-MicroDuck` | `MICRODUCK_ALLCOLL_MODEL=ext MICRODUCK_EXPERT_BC=0 MICRODUCK_ENTROPY_COEF=0.003 MICRODUCK_FALLSAFE_BANK=<kryakva>/training/data/fallsafe_bank_pred010_35_walkP5_1021.npz MICRODUCK_SIM_BUFFERS=75,350` | 4096 | 1999 |

[training/data](../data) — банки стартовых состояний: позы после падения (подъём), состояния в момент срабатывания защиты (защитное падение). `MICRODUCK_SIM_BUFFERS` — число контактов и ограничений на среду; для другой модели подбирается заново.
