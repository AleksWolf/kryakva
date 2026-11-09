# Servo console

A Python server and a browser page. On the left — a 3D model of the duck that follows the servo positions; on the right — 15 sliders by ID
with live reading of position, voltage, temperature, load, current and status bits. Based on `servo-web` from
fanhao375/microduck-replica (Apache-2.0), the model is our own.

## Connection

- From a computer (Windows or Linux): UC-01 over USB-C, port — COM on Windows, `/dev/ttyUSB0` on Linux;
  7.4 V servo power — into the UC-01 terminal block.
- On the duck: Zero 3W through its own servo port `/dev/ttyS2` (UART2, pins 8/10), power — from the duck's battery.

## Running on a computer

```
pip install -r requirements.txt
python server.py --port COM5                                   # Windows; port number is in Device Manager
python server.py --port /dev/ttyUSB0                           # Linux
python server.py --fake                                        # no servos: virtual servos, to look at the interface
```

Page — `http://127.0.0.1:8080`. Only one program can hold the port: the console and `servo_report.py` do not work at the same time.

## Running on the duck

Packages (once):

```
sudo apt-get install -y python3-starlette python3-uvicorn python3-websockets python3-serial
```

Copy the `software/servo_console` folder to the board into `/home/duck/servo_console`.

robotd holds the servo port, so stop it. When stopped, robotd releases torque on all servos, and the duck goes limp.
Stop only with the duck lying down:

```
sudo systemctl stop padd robotd
cp -r /home/duck/servo_console /tmp/ && cd /tmp/servo_console
python3 server.py --port /dev/ttyS2
```

The console runs from the copy in `/tmp`: its `logs/` journal is written to RAM, not to the card. Page — `http://<Zero address>:8080`.
Directions changed on the duck are stored in this copy; to keep them, copy `directions.json`
back to `/home/duck/servo_console`. When done — Ctrl+C and `sudo systemctl start robotd padd`.

While the console runs on the duck, do not run the console on a computer through the UC-01 USB-C: there is one bus.

## Page

Language — English or Russian, the EN | RU switch at the top; the choice is remembered in the browser.

Top: link, online N/15, poll rate, bus voltage; port selection and "Connect".
Buttons: torque on/off for all; "Sliders = current positions" — before enabling torque, otherwise the servos jump to old goals;
"All to zero (2048)" — legs straight, head straight, only after zero calibration; "Animate head", "Rescan".

Each servo has: goal slider (0–4095, 2048 = zero, 11.4 counts per degree; yellow mark — actual position), angle, voltage,
temperature, load, current, torque checkbox, direction sign button. Panels below: zero calibration, single servo (registers 0–86,
reboot, ID change), motion (transition time, acceleration, stop), bus cycle test, log.

The sliders are limited to the joint limits from the model; the server clips any goal to the same limits and writes this to the log.

## Directions

`directions.json` — the sign of each servo; affects only the 3D view and does not touch the servo itself. After assembly check every joint:
+10° on the slider — the servo and the 3D view move the same way. If not — the ± button of that servo.

## Other scripts

- `new_servo.py` — ID and zero of a new servo, procedure — [hardware/servo_setup.md](../../hardware/servo_setup.md).
- `servo_report.py --port COMx` — reads the registers of all servos, writes nothing; report in `logs/`.
