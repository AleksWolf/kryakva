# Radxa Zero 3W setup

System — Armbian (kernel 6.1.115-vendor-rk35xx), boot from microSD. The user in the examples is `duck`.

## Image

`Armbian_26.8.1_Radxa-zero3_trixie_vendor_6.1.115_minimal.img.xz` from dl.armbian.com/radxa-zero3/Trixie_vendor_minimal.
Check it against the `.sha` file next to the image and write it to microSD with Armbian Imager; in its profile set Wi-Fi and the user `duck`.
Wi-Fi — a 5 GHz network from the start (section 8).

After the first boot, the board's address is in the router's device list. Log in from the computer: `ssh duck@<Zero address>`.
Copy repository folders to the board from the computer with scp, for example:

```
scp -r software/zero duck@<Zero address>:/home/duck/zero
```

Files copied from Windows lose the execute permission on scripts and programs. Restore it after copying the folder:

```
chmod +x /home/duck/zero/*.sh /home/duck/zero/*/*.sh                         # software/zero
chmod +x /home/duck/duck-bench/bin/* /home/duck/duck-bench/standalone/*.sh   # software/robotd
```

## 1. Ports, overlays and the amplifier service

Copy the [software/zero/](zero/) folder to the board into `/home/duck/zero` and run the `setup_zero.sh` script from there:

- installs the packages `i2c-tools alsa-utils v4l-utils wget gpiod gcc make device-tree-compiler dkms linux-headers-vendor-rk35xx`;
- builds `tof_check` — the rangefinder check, used by the duck page;
- in `/boot/armbianEnv.txt` enables the overlays `i2c3-m0` (IMU, pins 3/5), `uart2-m0` (servo bus, pins 8/10) and `radxa-zero3-rpi-camera-v2` (camera),
  removes the kernel console from the servo port (`console=display`, `serial-getty@ttyS2` disabled);
- builds and enables our own overlay `duck-bench.dts`: I2S3 for the amplifier and microphone, I2C4 and I2C5 for the rangefinder;
- installs the `duck-amp` service: holds the amplifier SD pin (pin 13) low and raises it only during playback;
- installs a rollback service: if within 5 minutes after reboot you do not run
  ```
  sudo systemctl disable --now bench-rollback
  ```
  the board restores the previous `/boot/armbianEnv.txt` and reboots.

After the script, reboot the board (`sudo reboot`), and within 5 minutes after boot log in over ssh and run the command above.

Check after reboot: `i2cdetect -y 5` sees the ToF (0x29), `i2cdetect -y 3` — the IMU (0x6a), `aplay -l` — the `duck-i2s3` card,
`ls /dev/video0` — the camera.

**ToF calibration.** Without it the rangefinder shows everything closer than 60 cm as nearer than it is. Do it once, on the assembled duck:

1. Put the duck 600 mm in front of a light, plain wall, with the ToF looking straight at it. Nothing else should be in its field of view.
2. Run the calibration; it saves `xtalk.bin` next to `tof_check`:
   ```
   cd /home/duck/zero && TOF_CAL="80 600" ./tof_check 5 20
   ```
3. Check: `TOF_XTALK=1 ./tof_check 5 20` shows a center of about 600 mm.

The duck page loads `xtalk.bin` by itself.

A sound file starts with at least 50 ms of silence and ends with zeros.

## 2. DualSense gamepad

The settings in this section are for the Sony DualSense gamepad.

This kernel has no `hid-playstation` driver. It is installed as a DKMS module from [software/zero/hid-playstation-dkms/](zero/hid-playstation-dkms/)
(armbian/linux-rockchip sources of the same kernel version):

```
cd /home/duck/zero
sudo cp -r hid-playstation-dkms /usr/src/hid-playstation-6.1.115-rk
sudo dkms install hid-playstation/6.1.115-rk
```

The module loads by itself when the gamepad connects (over USB or Bluetooth); no reboot needed.

Gamepad packets must go through the kernel, not through `bluetoothd`: otherwise `bluetoothd` constantly loads the CPU and the board heats up.
In `/etc/bluetooth/input.conf` set `UserspaceHID=false` and reboot the board. Restarting bluetooth instead of rebooting does not work:
the adapter hangs.

Bluetooth pairing — the `pair_ds.sh` script:

1. Reset the gamepad with a paperclip (hole on the back, ~5 s).
2. Enter pairing mode: Create + PS until fast blue blinking.
3. Gamepad MAC: while the gamepad blinks, run `bluetoothctl --timeout 30 scan on` on the board — the address is in the `DualSense Wireless Controller` line.
4. On the board, in `/home/duck/zero`: `sudo ./pair_ds.sh <gamepad MAC>` — the script enables `bondable`/`connectable`, finds the gamepad, does pair, trust, connect.

After that the gamepad connects by itself with the PS button. The system shows three devices: gamepad, Motion Sensors, Touchpad.

## 3. Camera

Camera auto exposure and white balance — Rockchip rkaiq built for this kernel ([software/zero/camera/](zero/camera/README.md)):

```
sudo /home/duck/zero/camera/setup_camera.sh
```

The script puts rkaiq into `/opt/rkaiq-v9`, downloads the Radxa camera tuning file into `/etc/iqfiles/` and enables the `rkaiq` service.
rkaiq picks up the camera when a stream starts: if a stream was running before rkaiq started, restart it.

## 4. Robot software

The software is Pollen's robotd (`pollen-robotics/microduck`) with patches for Feetech HD-1910 servos and the IMU on I²C.
The bundle is in [software/robotd/](robotd/README.md).

1. Copy the `software/robotd` folder to the board into `/home/duck/duck-bench`. It contains `bin/robotd`, `bin/robotctl`, `bin/padd`, `robotd.toml`,
   `policies/`, `hear/libonnxruntime.so`, `models/pickup_detector.onnx` and `standalone/`.
2. Install the `robotd` and `padd` services:
   ```
   sudo /home/duck/duck-bench/standalone/install.sh
   ```
   The script installs the services and moves the systemd journal to RAM. It enables and starts nothing.
3. Check by hand:
   ```
   sudo systemctl start robotd
   /home/duck/duck-bench/bin/robotctl --robot-socket /run/duck/robot.sock health --json
   sudo systemctl start padd
   ```
   On start, robotd releases torque on all 15 servos: the duck goes limp.
4. robotd tick journal: written to RAM, copied to the card into `/var/lib/duck/ticks/` every 60 s and at shutdown, at most 2 GB:
   ```
   sudo /home/duck/zero/tick_journal/install.sh
   ```
5. Start at boot:
   ```
   sudo systemctl enable robotd padd
   ```

Gamepad:

| Button | What it does |
|---|---|
| PS | stop: torque released on all servos |
| Options | the duck stands up and walks in place; when lying — first FallSafe, then stand-up |
| left stick | forward and back, sideways |
| right stick horizontal | turn |
| Create, hold 4 s | release torque and shut down the board |

If the gamepad link is lost, robotd releases torque by itself after about 2 s.

## 5. Duck page

Status and control page: `http://<Zero address>:8082`. Runs on top of robotd, installed after it —
[software/duck_page](duck_page/README.md).

## 6. Servo console

The servo console can also run on the duck, on the servo port `/dev/ttyS2`, — [software/servo_console](servo_console/README.md).

## 7. microSD card

To keep the card from getting corrupted on a sudden power loss, logs are written to RAM:

- `/var/log` — in zram (Armbian default, the `armbian-ramlog` service);
- `/tmp` — tmpfs;
- systemd journal — in RAM (moved there by robotd's `install.sh`, section 4);
- robotd tick journal — in RAM, copied to the card compressed every 60 s and at shutdown (section 4).

Check: `findmnt /var/log /tmp` — zram and tmpfs; `journalctl --header | grep "File path"` — the path is in `/run/log/journal`.

## 8. Wi-Fi

The duck's network — 5 GHz: when the gamepad is running, 2.4 GHz Wi-Fi drops out. The simplest way is to set a 5 GHz network right in the Armbian Imager profile;
on a ready board — in `/etc/netplan/*.yaml`.

## 9. Shutdown

The servos are powered from the battery and keep torque even after the Zero shuts down. So first release torque on the servos, then `sudo poweroff`.
Release torque only with the duck lying down — a standing duck will fall.
