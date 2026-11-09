#!/bin/sh
# Camera auto exposure and white balance: rkaiq (built from source, see README.md) and the Radxa IQ file for rpi-camera-v2.
set -e
cd "$(dirname "$0")"

sudo install -d /opt/rkaiq-v9
sudo install -m 0755 rkaiq_3A_server /opt/rkaiq-v9/
sudo install -m 0644 librkaiq.so /opt/rkaiq-v9/

# IQ file from the Radxa package rockchip-iqfiles-rk356x 0.2.6 (only the one file is taken, the package is not installed)
T=$(mktemp -d)
wget -q -O "$T/iq.deb" https://radxa-repo.github.io/bookworm/pool/main/r/rockchip-iqfiles/rockchip-iqfiles-rk356x_0.2.6_all.deb
echo "64e602c7f7091fbcd452d2415e2aa1bc9d6b1460d20b3bae447d261f2c4b094a  $T/iq.deb" | sha256sum -c -
dpkg-deb -x "$T/iq.deb" "$T/x"
sudo install -D -m 0644 "$T/x/usr/share/rockchip-iqfiles-rk356x/imx219_rpi-camera-v2_default.json" /etc/iqfiles/imx219_rpi-camera-v2_default.json
rm -rf "$T"

sudo cp rkaiq.service /etc/systemd/system/rkaiq.service
sudo systemctl daemon-reload
sudo systemctl enable --now rkaiq
