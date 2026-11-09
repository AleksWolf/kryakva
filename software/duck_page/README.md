# Duck page

Status and control page on the duck itself: `http://<Zero address>:8082`. Runs on top of the robot software (`robotd`):
reads the servos and the IMU through it and does not touch the servo bus itself.

Shows: battery voltage, servos (position, temperature, errors), IMU, ToF distance, camera check, CPU temperature,
a 3D model of the duck in its current pose, the log.

Buttons: "Stand up" — enable the policy (the duck stands up); "Relax" — release torque (a standing duck will fall); joystick — speed and turn while the page sends frames (no frame for 350 ms — stop);
while a DualSense is connected, the page joystick is off; "POWER OFF" — release torque and shut down the Zero;
camera and ToF switches.

Page language — English or Russian, the EN | RU switch at the top; the choice is remembered in the browser.

## Installation

Needs robotd from the [Zero setup](../zero_setup.md) (section 4) and `tof_check` from `/home/duck/zero` (section 1).

Copy the `software/duck_page` folder to the board into `/home/duck/duck_page` and install the service:

```
cd /home/duck/duck_page
sudo cp duckpage.service /etc/systemd/system/ && sudo systemctl enable --now duckpage
```

Writes only to the systemd journal.
