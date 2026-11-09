# -*- coding: utf-8 -*-
"""A new HD-1910 servo in the chain. A new servo always comes with ID 1; only one new servo on the bus at a time.

  python new_servo.py check   — who is on the bus (configured ones: ID and zero after a power cycle); for ID 1: factory report (servo_report),
                                register check against the model and a 60° step test (speed, delay, voltage), while there are < 3 measurements
                                or the speed spread is > 3 %; result — averages over all servos
  python new_servo.py setid N — ID 1 → N (a free ID from the Pollen table)
  python new_servo.py center N — offset 0, shaft to 2048, torque off (align the horn with the plate)
  python new_servo.py hold N  — the same, but torque STAYS on: the shaft holds 2048 while the horn screw is tightened
  python new_servo.py free N  — release torque (after hold)
  python new_servo.py save N  — 0x0B: current position = 2048, appended to the journal logs/ids_and_centers.md
Port — --port (default COM3)."""
import argparse, json, os, sys, time
import feetech, servo_report

HERE = os.path.dirname(os.path.abspath(__file__))
LOG = os.path.join(HERE, "logs")
STEPS = os.path.join(LOG, "steps.json")
JOURNAL = os.path.join(LOG, "ids_and_centers.md")
JOINTS = {10: "right_hip_yaw", 11: "right_hip_roll", 12: "right_hip_pitch", 13: "right_knee", 14: "right_ankle",
          20: "left_hip_yaw", 21: "left_hip_roll", 22: "left_hip_pitch", 23: "left_knee", 24: "left_ankle",
          30: "neck_pitch", 31: "head_pitch", 32: "head_yaw", 33: "head_roll", 34: "mouth"}
EXPECT = {33: 4, 21: 32, 22: 40, 23: 0, 19: 4, 13: 80, 84: 0, 85: 0, 86: 0}
EXPECT16 = {28: 500, 16: 1000}


def soff(o):
    return -(o & 0x7FFF) if o & 0x8000 else o


def put(b, S, addr, v, two=False):
    """Write with read-back: FeetechBus.write silently swallows a lost reply, and writes to 41/46 do get lost —
    the servo then runs with the old 41 = 20 / 46 = 1500 (≈ 247 counts in 0.5 s, the "sluggish" ~230)."""
    for _ in range(5):
        (b.write_u16 if two else b.write_u8)(S, addr, v)
        got = (b.read_u16 if two else b.read_u8)(S, addr)
        if got == v:
            return
        print(f"  ! #{S}: write {addr} = {v} lost (reads {got}), retrying")
    raise RuntimeError(f"#{S}: register {addr} was not written ({v})")


def step_test(b, S):
    """Six 60° steps without speed and acceleration limits; returns the averages."""
    import numpy as np                                      # only this test needs numpy
    p0 = b.read_u16(S, 56)
    b.write_u8(S, 41, 20); b.write_u16(S, 46, 1500); b.write_u16(S, 42, p0); b.write_u8(S, 40, 1)
    start = 2048 - 342
    b.write_u16(S, 42, start); time.sleep(1.5)
    put(b, S, 41, 0); put(b, S, 46, 10000, True)
    res = []
    try:
        for k in range(6):
            a, c = (start, start + 683) if k % 2 == 0 else (start + 683, start)
            time.sleep(0.4); rec = []
            t0 = time.perf_counter(); b.write_u16(S, 42, c)
            while time.perf_counter() - t0 < 0.5:
                s = b.state(S); rec.append((time.perf_counter() - t0, s["pos"], s["volt"]))
            R = np.array(rec); t, x, V = R[:, 0], (R[:, 1] - a) * np.sign(c - a), R[:, 2]
            i20, i80 = np.argmax(x >= 0.2 * 683), np.argmax(x >= 0.8 * 683)
            if x.max() < 0.8 * 683 or i80 <= i20 + 1:
                raise RuntimeError(f"step {a}→{c}: in 0.5 s it reached only {x.max():.0f} of 683 counts — is something holding the horn?")
            res.append((np.polyfit(t[i20:i80 + 1], x[i20:i80 + 1], 1)[0] * 2 * np.pi / 4096,
                        1000 * t[np.argmax(x >= 3)], 1000 * t[np.argmax(x >= 680)], V.min(), V.max()))
    finally:
        b.write_u8(S, 41, 20); b.write_u16(S, 46, 1500); b.write_u16(S, 42, b.read_u16(S, 56)); b.write_u8(S, 40, 0)
    r = np.array(res)
    return {"speed": float(r[:, 0].mean()), "delay_ms": float(r[:, 1].mean()), "t60_ms": float(r[:, 2].mean()),
            "vmin": float(r[:, 3].min()), "vmax": float(r[:, 4].max())}


def check(b, port):
    online = [i for i in sorted(JOINTS) if b.ping(i) is not None]
    for i in online:
        print(f"  #{i} {JOINTS[i]}: position {b.read_u16(i, 56)}, offset {soff(b.read_u16(i, 31)):+d}, torque {b.read_u8(i, 40)}")
    if b.ping(1) is None:
        print("No new servo (ID 1) on the bus." + ("" if online else " Nobody answers — power/cable?")); return
    print("New servo ID 1: factory report…")
    servo_report.run(b, [1], port, os.path.join(HERE, "logs"))
    raw = b.dump(1)["raw"]
    bad = [f"{a}={raw[a]} (expected {v})" for a, v in EXPECT.items() if raw[a] != v]
    bad += [f"{a}={raw[a] | raw[a + 1] << 8} (expected {v})" for a, v in EXPECT16.items() if (raw[a] | raw[a + 1] << 8) != v]
    print(f"  firmware {raw[0]}.{raw[1]}, servo {raw[3]}.{raw[4]}; registers: {'all as in the model' if not bad else 'MISMATCHES: ' + ', '.join(bad)}")
    steps = json.load(open(STEPS, encoding="utf-8")) if os.path.exists(STEPS) else {}
    print(f"  60° step ×6 (the horn will turn), {len(online) + 1} servos in the chain…")
    try:
        r = step_test(b, 1); stall = ""
    except RuntimeError as e:                              # on some new servos the first series is sluggish (acceleration ~44 units), a repeat is normal; cause unknown
        stall = str(e); print("  SLUGGISH:", stall, "— repeating"); r = step_test(b, 1)
    r["chain"] = len(online) + 1; r["first_stall"] = stall
    steps[f"new-{time.strftime('%H%M%S')}"] = r          # the key is renamed to the ID on setid
    os.makedirs(LOG, exist_ok=True)
    json.dump(steps, open(STEPS, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print(f"  speed {r['speed']:.2f} rad/s, delay {r['delay_ms']:.1f} ms, 60° in {r['t60_ms']:.0f} ms, voltage {r['vmin']:.1f}–{r['vmax']:.1f} V")
    print("  along the chain (servos in the chain: speed rad/s, delay ms, 60° ms, min voltage V):")
    for k, v in steps.items():
        print(f"    {k:>16} — {v.get('chain', '?')}: {v['speed']:.2f}, {v['delay_ms']:.1f}, {v['t60_ms']:.0f}, {v['vmin']:.1f}")
    free = [i for i in sorted(JOINTS) if i not in online]
    print("  free IDs:", ", ".join(f"{i} {JOINTS[i]}" for i in free))


def main():
    sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(); ap.add_argument("cmd"); ap.add_argument("id", nargs="?", type=int); ap.add_argument("--port", default="COM3")
    a = ap.parse_args(); b = feetech.FeetechBus(a.port)
    if a.cmd == "check":
        check(b, a.port)
    elif a.cmd == "setid":
        assert a.id in JOINTS and b.ping(a.id) is None and b.ping(1) is not None, "need a free ID from the table and a servo on ID 1"
        print("ID 1 →", a.id, "ok" if b.set_id(1, a.id) and b.ping(1) is None else "ERROR")
        steps = json.load(open(STEPS, encoding="utf-8")) if os.path.exists(STEPS) else {}
        new = [k for k in steps if k.startswith("new-")]
        if new:
            steps[str(a.id)] = steps.pop(new[-1]); json.dump(steps, open(STEPS, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    elif a.cmd in ("center", "hold"):
        S = a.id; b.write_u8(S, 40, 0); b.unlock(S); b.write_u16(S, 31, 0); time.sleep(0.05); b.lock(S); time.sleep(0.05)
        p = b.read_u16(S, 56); b.write_u8(S, 41, 20); b.write_u16(S, 46, 1500); b.write_u16(S, 42, p); b.write_u8(S, 40, 1)
        b.write_u16(S, 42, 2048); t0 = time.time()
        while time.time() - t0 < 4 and abs(b.read_u16(S, 56) - 2048) > 2:
            time.sleep(0.05)
        if a.cmd == "center":
            b.write_u8(S, 40, 0)
            print(f"#{S}: offset {soff(b.read_u16(S, 31)):+d}, position {b.read_u16(S, 56)}, torque off — align the horn")
        else:
            # torque stays on after the port is closed: it is a servo register, held until power is removed
            print(f"#{S}: offset {soff(b.read_u16(S, 31)):+d}, position {b.read_u16(S, 56)}, "
                  f"torque ON — the shaft holds 2048, tighten the screw. Release: free {S}")
    elif a.cmd == "free":
        S = a.id; b.write_u8(S, 40, 0)
        print(f"#{S}: torque released, position {b.read_u16(S, 56)}")
    elif a.cmd == "save":
        S = a.id; assert b.read_u8(S, 40) == 0, "torque must be off"
        before = b.read_u16(S, 56); st = b.calibrate_to(S); time.sleep(0.3)
        off = soff(b.read_u16(S, 31)); pos = b.read_u16(S, 56); fw = b.read(S, 0, 2)[1]
        print(f"#{S}: before {before}, after {pos}, offset {off:+d}, status {st}")
        os.makedirs(LOG, exist_ok=True)
        with open(JOURNAL, "a", encoding="utf-8") as f:
            f.write(f"| {time.strftime('%d.%m.%Y')} | {S} | {JOINTS[S]} | {fw[0]}.{fw[1]} | {off:+d} | after power cycle — at the next check | — |\n")
    b.close()


if __name__ == "__main__":
    main()
