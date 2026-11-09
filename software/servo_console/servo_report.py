# -*- coding: utf-8 -*-
"""舵机体检：一条命令读出每颗舵机的保护设置和当前状态，存成报告。**只读，不写舵机任何寄存器。**

  python servo_report.py --port COM5                 # 默认 15 颗：20-24,30-34,10-14
  python servo_report.py --port COM5 --ids 21        # 总线上只接 21 号时
  python servo_report.py --port COM5 --scan-all      # 不知道 ID（新舵机出厂是 1）：扫 0–253

报告写在 logs/report-日期-时间.md（给人看）和同名 .json（每颗 0–86 寄存器原值）。
串口不能同时被调试台（server.py）或 FD 占着，先关掉它们。
判据按《调试记录》第 5 步：过热（4）、过流（8）、过压（1）三个保护都要开，没开标红。
"""
import argparse
import json
import os
import sys
import time

import feetech

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_IDS = "20-24,30-34,10-14"
NAMES = {20: "L hip yaw", 21: "L hip roll", 22: "L hip pitch", 23: "L knee", 24: "L ankle",
         30: "neck pitch", 31: "head pitch", 32: "head yaw", 33: "head roll", 34: "beak",
         10: "R hip yaw", 11: "R hip roll", 12: "R hip pitch", 13: "R knee", 14: "R ankle"}
# 卸载条件（19）/ 状态位（65）每一位的意思，两个寄存器同一套定义
BITS = {1: "voltage", 2: "encoder", 4: "overheat", 8: "overcurrent", 32: "overload"}
MUST = (4, 8, 1)          # 调试记录第 5 步：过热出厂开着，过流、过压要自己开


def parse_ids(text):
    ids = []
    for part in text.split(","):
        part = part.strip()
        if "-" in part:
            a, b = part.split("-")
            ids.extend(range(int(a), int(b) + 1))
        elif part:
            ids.append(int(part))
    return ids


def bits_text(v):
    on = [n for b, n in BITS.items() if v & b]
    return ", ".join(on) if on else "none"


def examine(bus, sid):
    """读一颗：PING → 导出 0–86 → 解码 → 判红黄。返回一行结果；哪一步读不到写在 step 里。"""
    row = {"id": sid, "name": NAMES.get(sid, ""), "online": False, "red": [], "yellow": [], "info": []}
    step = "PING"
    try:
        if bus.ping(sid) is None:
            row["red"].append("does not answer PING: no power, bad contact, wrong ID or a burnt driver")
            return row
        row["online"] = True
        step = "register dump 0–86"
        d = bus.dump(sid)
    except Exception as e:
        row["red"].append(f"failure at step “{step}”: {type(e).__name__}: {e}")
        return row
    v = {r["addr"]: r["value"] for r in d["rows"]}
    row["raw"] = d["raw"]
    s15 = feetech.sign15
    row.update({
        "firmware": f"{v[0]}.{v[1]}", "servo_ver": f"{v[3]}.{v[4]}",
        "unload": v[19], "status": v[65], "temp_c": v[63], "volt": v[62] / 10,
        "volt_range": (v[15] / 10, v[14] / 10), "temp_limit": v[13],
        # 38 / 35 的单位按 10 ms 换算，报告里原值也写上（单位没在真机上核过）
        "protect_ma": round(v[28] * 6.5), "overcurrent_raw": v[38], "protect_time_raw": v[35],
        "overload_pct": v[36], "max_torque": v[16], "torque_limit": v[48],
        "angle_limit": (v[9], v[11]), "offset": s15(v[31]), "pos": s15(v[56]), "goal": s15(v[42]),
        "current_ma": round(s15(v[69]) * 6.5), "load_pct": feetech.sign10(v[60]) / 10,
        "torque_on": v[40], "lock": v[55], "mode": v[33], "baud": v[6], "reply_level": v[8],
    })
    missing = [BITS[b] for b in MUST if not v[19] & b]
    if missing:
        row["red"].append(f"unload conditions 19 = {v[19]} (enabled: {bits_text(v[19])}), not enabled: {', '.join(missing)}")
    if not v[19] & 32:
        row["yellow"].append("overload protection (32) is not enabled: it unloads when the load stays above “overload torque” longer than “protection time” — extra insurance against jamming")
    if v[65]:
        row["red"].append(f"status bits 65 = {v[65]}: active alarm {bits_text(v[65])}")
    lo, hi = row["volt_range"]
    if not lo <= row["volt"] <= hi:
        row["red"].append(f"voltage {row['volt']:.1f} V outside the range set in the servo {lo:.1f}–{hi:.1f} V")
    if v[63] >= 60:
        row["yellow"].append(f"temperature {v[63]} °C, rather high (torque switches off at {v[13]} °C)")
    if abs(row["offset"]) > 200:
        row["yellow"].append(f"offset {row['offset']} counts ({row['offset'] * 360 / 4096:+.1f}°), large: the horn is mounted a few teeth or half a turn off")
    if v[33] != 4:
        row["yellow"].append(f"operating mode 33 = {v[33]}, per the fanhao375 debug log (调试记录) it should be 4 (pure position PD)")
    if (v[9], v[11]) == (0, 4095):
        row["info"].append("angle limits not set (0–4095): the servo will follow any command, including into a mechanical stop")
    return row


def report_md(rows, port, t):
    L = [f"# Servo report {t}", "", f"Port {port}, read-only, no register was written. Criteria — at the top of servo_report.py.", ""]
    online = [r for r in rows if r["online"]]
    reds = [r for r in rows if r["red"]]
    L.append(f"**On the line {len(online)}/{len(rows)}; with red items {len(reds)}**" + (f": {' '.join(str(r['id']) for r in reds)}" if reds else ""))
    L += ["", "| ID | joint | online | unload conditions 19 | status 65 | temperature | voltage | current | reading | offset | torque | firmware |",
          "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        if not r["online"]:
            L.append(f"| {r['id']} | {r['name']} | ✗ | | | | | | | | | |")
            continue
        L.append(f"| {r['id']} | {r['name']} | ✓ | {r['unload']}（{bits_text(r['unload'])}） | {r['status']}（{bits_text(r['status'])}） "
                 f"| {r['temp_c']} °C | {r['volt']:.1f} V | {r['current_ma']} mA | {r['pos']} | {r['offset']:+d} "
                 f"| {'on' if r['torque_on'] else 'off'} | {r['firmware']} |")
    L += ["", "## Notes per servo", ""]
    for r in rows:
        items = [f"- 🔴 {x}" for x in r["red"]] + [f"- 🟡 {x}" for x in r["yellow"]] + [f"- {x}" for x in r["info"]]
        if items:
            L += [f"**#{r['id']} {r['name']}**", *items, ""]
    L += ["## Protection settings (servos on the line)", "", "| ID | protection current | overcurrent protection time 38 | overload torque | protection time 35 | temperature limit | max torque 16 | torque limit 48 | angle limits |",
          "|---|---|---|---|---|---|---|---|---|"]
    for r in online:
        L.append(f"| {r['id']} | {r['protect_ma']} mA | {r['overcurrent_raw']} (if the step is 10 ms — {r['overcurrent_raw'] * 10} ms) | {r['overload_pct']} % "
                 f"| {r['protect_time_raw']} (if the step is 10 ms — {r['protect_time_raw'] * 10} ms) "
                 f"| {r['temp_limit']} °C | {r['max_torque']} | {r['torque_limit']} | {r['angle_limit'][0]}–{r['angle_limit'][1]} |")
    return "\n".join(L) + "\n"


def run(bus, ids, port, out_dir):
    t = time.strftime("%Y-%m-%d %H:%M:%S")
    rows = []
    for sid in ids:
        r = examine(bus, sid)
        if r["online"] or len(ids) <= 32:          # 扫 0–253 时只留在线的
            rows.append(r)
        mark = "✗" if not r["online"] else ("🔴" if r["red"] else ("🟡" if r["yellow"] else "✓"))
        if r["online"] or len(ids) <= 32:
            print(f"#{sid:<3} {r['name']:<18} {mark}  " + "; ".join(r["red"] + r["yellow"])[:160], flush=True)
    os.makedirs(out_dir, exist_ok=True)
    base = os.path.join(out_dir, "report-" + time.strftime("%Y%m%d-%H%M%S"))
    with open(base + ".md", "w", encoding="utf-8") as f:
        f.write(report_md(rows, port, t))
    with open(base + ".json", "w", encoding="utf-8") as f:
        json.dump({"time": t, "port": port, "servos": rows}, f, ensure_ascii=False, indent=1)
    return base, rows


def main(argv=None):
    ap = argparse.ArgumentParser(description="Servo report (read-only)")
    ap.add_argument("--port", required=True, help="serial port, e.g. COM5 / /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--ids", default=DEFAULT_IDS)
    ap.add_argument("--scan-all", action="store_true", help="scan 0–253, only the answering ones go into the report")
    ap.add_argument("--out", default=os.path.join(HERE, "logs"))
    a = ap.parse_args(argv)
    ids = list(range(0, 254)) if a.scan_all else parse_ids(a.ids)
    try:
        bus = feetech.FeetechBus(a.port, a.baud)
    except Exception as e:
        raise SystemExit(f"Cannot open port {a.port}: {type(e).__name__}: {e} (if the console or FD holds it — close them)")
    try:
        base, rows = run(bus, ids, a.port, a.out)
    finally:
        bus.close()
    reds = [r["id"] for r in rows if r["red"]]
    print(f"\nOn the line {sum(r['online'] for r in rows)}/{len(rows)}; red items: {reds or 'none'}")
    print(f"Report: {base}.md\nRaw registers: {base}.json")
    return 1 if reds else 0


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    sys.exit(main())
