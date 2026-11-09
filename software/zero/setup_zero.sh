#!/bin/sh
# Sets up the Zero: packages, tof_check build, boot overlays (servo UART, IMU I2C, camera, duck-bench), amplifier service.
# Rollback: if within 5 minutes after the reboot you do not run
#   sudo systemctl disable --now bench-rollback
# the Zero puts back the previous armbianEnv.txt and reboots (in case Wi-Fi is lost).
set -e
cd "$(dirname "$0")"

sudo apt-get update
sudo apt-get install -y i2c-tools alsa-utils v4l-utils wget gpiod gcc make device-tree-compiler dkms linux-headers-vendor-rk35xx
make
dtc -@ -q -I dts -O dtb -o duck-bench.dtbo duck-bench.dts

[ -f /boot/armbianEnv.txt.before_bench ] || sudo cp /boot/armbianEnv.txt /boot/armbianEnv.txt.before_bench
sudo mkdir -p /boot/overlay-user
sudo cp duck-bench.dtbo /boot/overlay-user/
set_env() {
	grep -q "^$1=" /boot/armbianEnv.txt \
		&& sudo sed -i "s|^$1=.*|$1=$2|" /boot/armbianEnv.txt \
		|| echo "$1=$2" | sudo tee -a /boot/armbianEnv.txt >/dev/null
}
# servo bus UART2 (pins 8/10), IMU I2C3 (pins 3/5), camera rpi-camera-v2; kernel console off the servo UART
set_env overlay_prefix rk3568
set_env overlays "i2c3-m0 uart2-m0 radxa-zero3-rpi-camera-v2"
set_env console display
set_env user_overlays duck-bench
sudo systemctl mask serial-getty@ttyS2

# amplifier service: SD (pin 13) low, high only while sound is playing
sudo cp duck_amp.py /usr/local/bin/duck_amp.py
sudo cp duck-amp.service /etc/systemd/system/duck-amp.service
sudo systemctl enable duck-amp.service

sudo tee /etc/systemd/system/bench-rollback.service >/dev/null <<'END'
[Unit]
Description=Roll back the boot overlays if not confirmed
After=multi-user.target

[Service]
Type=simple
ExecStart=/bin/sh -c 'sleep 300; cp /boot/armbianEnv.txt.before_bench /boot/armbianEnv.txt; systemctl disable bench-rollback; sync; reboot'

[Install]
WantedBy=multi-user.target
END
sudo systemctl daemon-reload
sudo systemctl enable bench-rollback
cat /boot/armbianEnv.txt
