# Политики

Три политики ONNX для утки на HD-1910. Обучены на моделях MJCF из [training/model](../model) рецептами из [training/patches](../patches/README.ru.md).

| Файл | Умение | Команда cmd[0..2] |
|---|---|---|
| [walk_hd1910_p5_v2_iter7992_home5.onnx](walk_hd1910_p5_v2_iter7992_home5.onnx) | ходьба | vx, vy, поворот; голова — 4 цели |
| [standup_hd1910_p5_v1_iter2998_home5.onnx](standup_hd1910_p5_v1_iter2998_home5.onnx) | подъём с пола на ноги | нули |
| [fallsafe_hd1910_p5_v1_iter1999_home5.onnx](fallsafe_hd1910_p5_v1_iter1999_home5.onnx) | защитное падение | нули |

Ходьба и подъём работают на утке.

## Общее для всех

- Вход 61: угловая скорость, гравитация, положения и скорости 14 суставов, прошлое действие, команда 3, голова 4, корпус 6.
  Выход 14 — порядок суставов в `names` файла [walk_hd1910_p5_v2_reference_trace.npz](walk_hd1910_p5_v2_reference_trace.npz).
- Сервы: регистры ОЗУ 50 = 5 (P), 51 = 0 (D) у всех 14 серв ног и шеи, с проверкой чтением; EEPROM не трогается.
- Цель: `raw = clip(home + 1.0 · a, clip_lo, clip_hi)`; `home`, `clip_lo`, `clip_hi` — из эталона.
- Фильтр каждый такт 20 мс: `target = target_prev + 0.7 · (raw − target_prev)`. При смене политики фильтр продолжает с текущей цели.

[walk_hd1910_p5_v2_reference_trace.npz](walk_hd1910_p5_v2_reference_trace.npz) — 500 тактов ходьбы для численной проверки рантайма: obs, a, target_raw, target, q, cmd,
names, home, alpha, clip_lo, clip_hi, scale, dt.

## Переходы

- Ходьба → FallSafe, когда наклон + 0.1 с × скорость наклона > 35°. FallSafe 2 с с нулевыми командами → StandUp, если утка не стоит.
  StandUp → ходьба, когда стоит 0.5 с (наклон < 10°, корпус выше 105 мм).
