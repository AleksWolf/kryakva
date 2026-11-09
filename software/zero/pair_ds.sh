#!/bin/bash
# Pairs a DualSense with the Zero 3W. Run on the board: sudo ./pair_ds.sh <pad MAC>
# Before: the pad in pairing mode (Create + PS until fast blue blinking).
M=${1:?usage: sudo ./pair_ds.sh <pad MAC>}
bluetoothctl remove "$M" >/dev/null 2>&1
btmgmt bondable on >/dev/null; btmgmt connectable on >/dev/null
btmgmt info | grep "current settings"
bluetoothctl --timeout 400 scan on > /tmp/scan.log 2>&1 &
echo "$(date +%T) scanning up to 6 min"
for i in $(seq 1 180); do bluetoothctl devices | grep -q "$M" && break; sleep 2; done
bluetoothctl devices | grep -q "$M" || { echo "$(date +%T) not found"; exit 1; }
echo "$(date +%T) found: $(bluetoothctl devices | grep "$M")"
echo "--- pair (agent NoInputNoOutput):"
{ echo "agent off"; sleep 1; echo "agent NoInputNoOutput"; sleep 1; echo "default-agent"; sleep 1; echo "pair $M"; sleep 25; echo "info $M"; sleep 2; echo "quit"; } \
  | bluetoothctl 2>&1 | sed "s/\x1b\[[0-9;]*m//g" | grep -vE "RSSI|NEW\] Device|DEL\] Device|ManufacturerData|^\s*$"
echo "--- trust+connect:"
{ echo "trust $M"; sleep 2; echo "connect $M"; sleep 10; echo "info $M"; sleep 2; echo "quit"; } \
  | bluetoothctl 2>&1 | sed "s/\x1b\[[0-9;]*m//g" | grep -vE "RSSI|NEW\] Device|DEL\] Device|ManufacturerData|^\s*$"
sleep 3
echo "--- inputs:"; grep -E "^N:|^H:" /proc/bus/input/devices | grep -i -A1 "dualsense"
echo "--- journal bluetooth:"; journalctl -u bluetooth --no-pager -n 30 | sed "s/^.*bluetoothd\[[0-9]*\]: //" | grep -iE "input|hid|bond|refus|$M" | tail -8
echo "$(date +%T) done"
