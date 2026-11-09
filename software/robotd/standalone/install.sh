#!/usr/bin/env bash
# The stand-alone duck's services (duck-brain P25, README.md here): robotd.service and padd.service
# for the bundle in DIR, and the journal in RAM. Installs, does NOT enable or start anything.
#
#   sudo ./install.sh [DIR]          # DIR: default /home/duck/duck-bench
#   sudo ./install.sh --uninstall
#
# DIR must hold bin/robotd, bin/robotctl, bin/padd, robotd.toml, policies/, hear/libonnxruntime.so.
# robotd.toml is read, never written.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
UNITS=${DUCK_UNITS:-/etc/systemd/system}                      # overridable for a test only
JOURNALD=${DUCK_JOURNALD:-/etc/systemd/journald.conf.d/50-duck.conf}
die() { echo "!! $*" >&2; exit 1; }
[ "$(id -u)" = 0 ] || [ -n "${DUCK_UNITS:-}" ] || die "run as root: sudo $0 $*"

if [ "${1:-}" = "--uninstall" ]; then
    systemctl disable --now padd.service robotd.service 2>/dev/null || true
    rm -f "$UNITS/robotd.service" "$UNITS/padd.service"
    systemctl daemon-reload
    echo "removed robotd.service and padd.service ($JOURNALD left as it is)"
    exit 0
fi

DIR=$(realpath -m "${1:-/home/duck/duck-bench}")
for f in bin/robotd bin/robotctl bin/padd robotd.toml hear/libonnxruntime.so; do
    [ -e "$DIR/$f" ] || die "missing $DIR/$f"
done
[ -d "$DIR/policies" ] || die "missing $DIR/policies/"
for b in robotd robotctl padd; do
    [ -x "$DIR/bin/$b" ] || die "$DIR/bin/$b is not executable"
done
case "$DIR" in *[[:space:]\#\&]*) die "DIR with spaces, # or &: not supported" ;; esac
grep -q '^driver = "feetech"' "$DIR/robotd.toml" || die "$DIR/robotd.toml: [bus] driver is not feetech"

# Pollen's services, if anything from that stack is here: two robotd on one bus would fight.
for u in robotd padd; do
    if [ -e "/lib/systemd/system/$u.service" ] || [ -e "/usr/lib/systemd/system/$u.service" ]; then
        die "a packaged $u.service exists (Pollen's stack?) — not overwriting it"
    fi
done

for u in robotd padd; do
    sed "s#@DIR@#$DIR#g" "$HERE/$u.service" > "$UNITS/$u.service.duck-new"
    mv "$UNITS/$u.service.duck-new" "$UNITS/$u.service"
    chmod 0644 "$UNITS/$u.service"
    echo "installed $UNITS/$u.service"
done

# The journal in RAM: robotd and padd log there, and nothing goes to the card.
if grep -hsq '^Storage=volatile' /etc/systemd/journald.conf /etc/systemd/journald.conf.d/*.conf "$JOURNALD"; then
    echo "journald: already Storage=volatile"
else
    mkdir -p "$(dirname "$JOURNALD")"
    install -m 0644 "$HERE/journald-duck.conf" "$JOURNALD"
    systemctl restart systemd-journald
    echo "journald: Storage=volatile, 64 MB ($JOURNALD) — the journal is in RAM from now, lost at power-off"
fi

systemctl daemon-reload
systemd-analyze verify "$UNITS/robotd.service" "$UNITS/padd.service" 2>&1 | grep -v "^$" || true

echo
echo "Not enabled, not started. Check first, by hand:"
echo "  sudo systemctl start robotd       # comes up limp (torque 0 on all 15)"
echo "  $DIR/bin/robotctl --robot-socket /run/duck/robot.sock health --json | grep -E '\"torque\"|\"ready\"'"
echo "  sudo systemctl start padd         # pad on (PS): Options = stand up; PS = stop"
echo "  journalctl -u robotd -u padd -f"
echo "Then, to start at boot:  sudo systemctl enable robotd padd"
echo "Paired pads BlueZ knows of:"
bluetoothctl devices Paired 2>/dev/null | sed 's/^/  /' || echo "  (bluetoothctl not answering)"
