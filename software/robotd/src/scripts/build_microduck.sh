#!/usr/bin/env bash
# Our build of Pollen's daemons (AGENTS.md: changes to microduck only as patches): the pinned
# commit, patches/microduck/*.patch on top, Pollen's tests, then the build.
#
#   scripts/build_microduck.sh                 # debug build for this machine (robotd --sim)
#   TARGET=aarch64 scripts/build_microduck.sh  # release for the duck (cargo zigbuild, glibc 2.31)
#
# Work tree: $WORK (default ../microduck-duck), a clone of the clone next to us
# (scripts/setup_cloud.sh) — that one is never touched. Binaries: $WORK/target/<...>/{robotd,tofd,robotctl}.
# The P4–P6 tests that need ONNX Runtime and our policies run when ORT_DYLIB_PATH is set.
#
# The pin moved from f0d934e to 95a7e91 (Pollen main of 13.10.2026) with P9; ../microduck needs the
# commit (`git -C ../microduck fetch origin`), its checkout stays where it is.
# P11–P13 (16.10.2026): the Feetech HD-1910 bus (new crate `feetech`), the host IMU, the tick journal.
set -euo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"
MD="${MICRODUCK:-$HERE/../microduck}"
WORK="${WORK:-$HERE/../microduck-duck}"
PIN="${PIN:-95a7e91}"          # the commit the patches are made against
TARGET="${TARGET:-host}"

rm -rf "$WORK"
git clone -q "$MD" "$WORK"
git -C "$WORK" checkout -q -b duck "$PIN"
for p in "$HERE"/patches/microduck/*.patch; do
    git -C "$WORK" -c user.name=duck-brain -c user.email=duck-brain@localhost am -q "$p"
    echo "applied $(basename "$p")"
done
cd "$WORK"
# Pollen's suite as Pollen runs it, without ONNX Runtime: two of its tests expect none (they fail
# the same on the unpatched commit when it is there).
env -u ORT_DYLIB_PATH cargo test -q -p feetech -p duck-control -p robotd-params -p robotd -p robotctl -p tof -p duck-ipc-proto -p btd -p updater 2>&1 | grep -E "^test result|FAILED|^error" || true
if [ -n "${ORT_DYLIB_PATH:-}" ]; then
    DUCK_BRAIN_POLICIES="$HERE/policies" cargo test -q -p robotd -- --ignored duck_brain 2>&1 | grep -E "^test result|FAILED|^error" || true
fi
if [ "$TARGET" = aarch64 ]; then
    cargo zigbuild --release --target aarch64-unknown-linux-gnu.2.31 -p robotd -p tof -p robotctl
    echo "built: $WORK/target/aarch64-unknown-linux-gnu/release/{robotd,tofd,robotctl}"
else
    cargo build -p robotd -p tof -p robotctl
    echo "built: $WORK/target/debug/{robotd,tofd,robotctl}"
fi
