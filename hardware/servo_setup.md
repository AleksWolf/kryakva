# Servos: ID and zero

Before assembly, one servo at a time. You need UC-01 (via USB-C to the computer), 7.4 V power into the UC-01 screw terminal, the printed plate
[cad/calibration_plate.step](../cad/calibration_plate.step) and the servo console [software/servo_console](../software/servo_console/README.md).

## Servo IDs

| Servo | ID |
|---|---|
| right leg: hip yaw, hip roll, hip pitch, knee, ankle | 10, 11, 12, 13, 14 |
| left leg: the same | 20, 21, 22, 23, 24 |
| neck pitch, head pitch, head yaw, head roll, beak | 30, 31, 32, 33, 34 |

## Procedure

A new servo answers with ID 1, so connect them one at a time: power off, connect the servo, power on.

From the folder [software/servo_console](../software/servo_console/README.md):

```
python new_servo.py check       # servo found, factory registers OK
python new_servo.py setid N     # N is the ID from the table
python new_servo.py center N    # shaft to the middle (2048), torque off
python new_servo.py save N      # save the zero
```

Default port is COM3; for another one use `--port COMx`.

Between `center` and `save`, align the horn with the calibration plate. After `save`, write its ID on the servo.

To read the registers of all servos on the bus without writing anything: `python servo_report.py --port COMx`.

## Registers

The factory P/D (32/40) in EEPROM are not changed. The policies run with P = 5, D = 0 in servo RAM ([training/policies](../training/policies/README.md)).
Over-current protection is off from the factory, overheat protection (80 °C) is on.

## Horns

The rear passive horns are removed on servos 10–14, 20–24, 32, 33, 34. On servos 30 and 31 the rear discs stay.
