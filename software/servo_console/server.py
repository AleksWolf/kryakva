# -*- coding: utf-8 -*-
"""飞特舵机网页调试台：串口 + 网页滑块 + 3D 鸭子跟着动。

用法：
  python server.py --port COM5                      # Windows，URT-2
  python server.py --port /dev/ttyUSB0              # 板子上，URT-2 插 USB
  python server.py --port /dev/ttyS2                # 板子上，走半双工转接板（先 sudo systemctl stop robotd）
  python server.py --fake                           # 没舵机，看界面
然后浏览器开 http://<主机>:8080
"""
import argparse
import asyncio
import contextlib
import json
import math
import os
import struct
import sys
import tempfile
import threading
import time
import traceback

import uvicorn
from starlette.applications import Starlette
from starlette.responses import FileResponse, JSONResponse
from starlette.routing import Route, WebSocketRoute
from starlette.websockets import WebSocketDisconnect

import feetech

HERE = os.path.dirname(os.path.abspath(__file__))

# 调试台版本。改了前端或后端就加一：index.html 里的 PAGE_VERSION 要跟这里一样，
# 页面连上后会比对，不一样就提示"页面是旧的，Ctrl+F5"。
VERSION = "0.15.9"
DEFAULT_IDS = "20-24,30-34,10-14"
JOINT_NAMES = {
    20: "left_hip_yaw", 21: "left_hip_roll", 22: "left_hip_pitch", 23: "left_knee", 24: "left_ankle",
    30: "neck_pitch", 31: "head_pitch", 32: "head_yaw", 33: "head_roll", 34: "mouth",
    10: "right_hip_yaw", 11: "right_hip_roll", 12: "right_hip_pitch", 13: "right_knee", 14: "right_ankle",
}


def parse_ids(text):
    ids = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            a, b = part.split("-")
            ids += list(range(int(a), int(b) + 1))
        else:
            ids.append(int(part))
    return ids


class FakeBus:
    """没舵机时的替身（模拟模式）：虚拟舵机按真舵机的速度 / 加速度寄存器走梯形曲线到目标，
    所以滑块、回零、灵动在 3D 里的快慢跟真机一样。扭矩默认开，没接串口也能直接看它动；
    扭矩关了就停在原地（没有重力，不会往下掉）。"""

    MAX_SPEED = 3000                  # 步/秒：HD-1910 满速约这么多，跟 MAX_SPEED_REG 一致
    ACC_UNIT = 8.7 * 4096 / 360       # 寄存器 41 一个单位 = 8.7°/s²，换成 步/s²
    ACC_MAX = 254 * ACC_UNIT          # 41 写 0 = 不限加速度，按最大算

    def __init__(self, ids):
        self.pos = {i: 2048.0 for i in ids}
        self.vel = {i: 0.0 for i in ids}
        self.goal = {i: 2048 for i in ids}
        self.speed_reg = {i: 0 for i in ids}   # 46：0 = 不限速
        self.acc_reg = {i: 0 for i in ids}     # 41：0 = 不限加速度
        self.torque = {i: 1 for i in ids}
        self.offset = {i: 0 for i in ids}
        self.locked = {i: 1 for i in ids}   # 出厂锁着
        self.regs = {}                      # 其它寄存器随便写随便读，(id, 地址) -> 值
        self.stats = {"tx": 0, "rx_ok": 0, "timeout": 0, "bad_checksum": 0, "bad_id": 0}
        self._t = time.monotonic()

    def _advance(self):
        """按墙钟推进每颗虚拟舵机：朝目标加速，离目标近了按 v² = 2·a·d 减速，跟舵机自己的梯形曲线一样。
        写寄存器之前先推进一次：不然上次读到这次写之间的时间，会按新目标补走。"""
        now = time.monotonic()
        dt, self._t = min(0.2, now - self._t), now
        if dt <= 0:
            return
        for i in self.pos:
            if not self.torque[i]:
                self.vel[i] = 0.0
                continue
            err = self.goal[i] - self.pos[i]
            if abs(err) < 0.5 and abs(self.vel[i]) < 1:
                self.pos[i], self.vel[i] = float(self.goal[i]), 0.0
                continue
            limit = self.speed_reg[i] * SPEED_UNIT
            vmax = limit if 0 < limit < self.MAX_SPEED else self.MAX_SPEED
            amax = self.acc_reg[i] * self.ACC_UNIT if self.acc_reg[i] else self.ACC_MAX
            want = math.copysign(min(vmax, math.sqrt(2 * amax * abs(err))), err)
            dv = max(-amax * dt, min(amax * dt, want - self.vel[i]))
            self.vel[i] += dv
            step = self.vel[i] * dt
            if step * err > 0 and abs(step) >= abs(err):   # 朝着目标走、这一步就到：停在目标上。背着目标（刚换向）不算
                self.pos[i], self.vel[i] = float(self.goal[i]), 0.0
            else:
                self.pos[i] += step

    def ping(self, sid):
        return 0 if sid in self.pos else None

    def states(self, ids):
        self._advance()
        out = {}
        for i in ids:
            if i not in self.pos:
                out[i] = None
                continue
            pos = int(round(self.pos[i]))
            out[i] = {"err": 0, "pos": pos, "speed": int(round(self.vel[i])), "load": 0, "volt": 7.9, "temp": 31,
                      "status": 0, "moving": int(abs(self.goal[i] - pos) > 2 or abs(self.vel[i]) > 1),
                      "goal": self.goal[i], "current_ma": 0.0, "status_text": ""}
        self.stats["tx"] += 1
        self.stats["rx_ok"] += len(ids)
        return out

    def write_u16(self, sid, addr, v):
        # 42 目标、46 速度、31 偏移跟真舵机一样是符号-幅值（BIT15 = 负），server 用 enc_sm 编好再写
        self._advance()
        v &= 0xFFFF
        self.regs[(sid, addr)] = v
        if addr == 42:
            self.goal[sid] = feetech.sign15(v)
        elif addr == 46:
            self.speed_reg[sid] = v & 0x7FFF
        elif addr == 31:
            off = feetech.sign15(v)
            self.pos[sid] += off - self.offset[sid]       # 读数 = 原始 + 偏移（OFFSET_SIGN = +1）
            self.offset[sid] = off
        return 0

    def write_u8(self, sid, addr, v):
        self._advance()
        if addr == 41:
            self.acc_reg[sid] = v & 0xFF
        if addr == 55:
            self.locked[sid] = 1 if v else 0
        elif addr != 40:
            self.regs[(sid, addr)] = v & 0xFF
        if addr == 40 and v != 128:          # 写 128 跟 HD-1910 一样：回成功，什么都不做
            self.torque[sid] = 1 if v == 1 else 0
        return 0

    def calibrate_to(self, sid, value=None):
        self._advance()
        value = 2048 if value is None else value
        self.offset[sid] += value - int(round(self.pos[sid]))
        self.pos[sid], self.vel[sid] = float(value), 0.0
        return 0

    def read_u8(self, sid, addr):
        return {40: self.torque[sid], 55: self.locked[sid]}.get(addr, self.regs.get((sid, addr), 0))

    def read_u16(self, sid, addr):
        self._advance()
        return {31: enc_sm(self.offset[sid]), 56: enc_sm(int(round(self.pos[sid]))),
                42: enc_sm(self.goal[sid])}.get(addr, self.regs.get((sid, addr), 0))

    def sync_write(self, addr, length, id_values):
        self._advance()
        for sid, data in id_values:
            if sid not in self.pos:
                continue
            if addr == 42:
                self.goal[sid] = feetech.sign15(struct.unpack("<H", bytes(data[:2]))[0])
            if addr == 41:                   # 加速度（+ 目标位置 + 时间 + 速度）
                self.acc_reg[sid] = data[0]
                if length >= 3:
                    self.goal[sid] = feetech.sign15(struct.unpack("<H", bytes(data[1:3]))[0])
                if length >= 7:
                    self.speed_reg[sid] = struct.unpack("<H", bytes(data[5:7]))[0] & 0x7FFF   # 符号-幅值，取幅值
            if addr == 46:
                self.speed_reg[sid] = struct.unpack("<H", bytes(data[:2]))[0] & 0x7FFF
            if addr == 40:
                self.torque[sid] = data[0]

    def sync_read(self, ids, addr, n):
        out = {i: (0, bytes([self.torque[i]]) + bytes(n - 1) if addr == 40 else bytes(n)) if i in self.pos else None
               for i in ids}
        return out

    def dump(self, sid):
        raw = [0] * feetech.DUMP_END
        raw[5], raw[8], raw[21], raw[22], raw[33], raw[55] = sid, 1, 32, 32, 4, self.locked.get(sid, 1)
        if sid in self.pos:
            off, pos = enc_sm(self.offset[sid]), enc_sm(int(round(self.pos[sid])))
            raw[31], raw[32], raw[40], raw[56], raw[57] = off & 0xFF, off >> 8, self.torque[sid], pos & 0xFF, pos >> 8
        rows = [{"addr": a, "name": feetech.REGISTERS[a][0], "area": feetech.REGISTERS[a][2],
                 "size": feetech.REGISTERS[a][1], "value": raw[a]} for a in sorted(feetech.REGISTERS)]
        return {"rows": rows, "raw": raw}

    def read(self, sid, addr, n):
        if addr == 0 and n == 5:
            return 0, bytes([3, 10, 0, 3, 11])
        return 0, bytes(n)

    def reboot(self, sid):
        pass

    def unlock(self, sid):
        self.locked[sid] = 0
        return 0

    def lock(self, sid):
        self.locked[sid] = 1
        return 0

    def set_id(self, old, new):
        self.pos[new] = self.pos.pop(old, 2048.0)
        self.vel[new] = self.vel.pop(old, 0.0)
        self.goal[new] = self.goal.pop(old, 2048)
        self.speed_reg[new] = self.speed_reg.pop(old, 0)
        self.acc_reg[new] = self.acc_reg.pop(old, 0)
        self.torque[new] = self.torque.pop(old, 1)
        self.offset[new] = self.offset.pop(old, 0)
        self.locked[new] = self.locked.pop(old, 1)
        return True

    def close(self):
        pass


BUS = None
PORT = None        # 当前串口名；None = 假总线
BAUD = 1_000_000
IDS = []
PRESENT = []
CLIENTS = set()
LOG = []           # [{"n": 序号, "t": "时:分:秒", "cat": 分类, "level": info/warn/error, "msg": {"en": 正文, "ru": 正文}}]
LOG_N = 0
LOG_DIR = os.path.join(HERE, "logs")
LOG_CATS = ["system", "bus", "motion", "calibration", "registers", "direction"]   # 页面自己还有一类 page
LEVEL_TAG = {"info": "", "warn": "[warning]", "error": "[error]", "debug": "[debug]"}
_log_io = threading.Lock()
STREAM_HZ = 50
FAKE_STREAM_HZ = 30    # 模拟模式不占总线，推快一点，3D 动起来顺
SPEED_UNIT = 1.0   # 速度寄存器 46 一个单位 = 多少步/秒：相位 18 BIT2=1 → 1（0.0146 rpm），BIT2=0 → 50（0.732 rpm）
MAX_SPEED_REG = 3000   # "不限速"写多少：相位 BIT3=1 时 0 = 最快；BIT3=0 时 0 = 停，得写个大数（HD-1910 满速约 3000 步/秒）
SLIDER_ACC, SLIDER_SPEED = 5, 57   # slider: gentle motion; after a smooth move, release leaves acceleration 0 and unlimited speed, and the servo jerks on every goal
STREAM_LAST = {}       # goals_stream 上一帧给每颗的目标，用来算这一帧该多快


def T(en, ru):
    """发给页面的文字两种语言都带：{"en": ..., "ru": ...}，页面按选的语言显示；日志文件和控制台用英文。"""
    return {"en": en, "ru": ru}


def _en(m):
    return m["en"] if isinstance(m, dict) else str(m)


def _ru(m):
    return m["ru"] if isinstance(m, dict) else str(m)


def tj(*parts, sep=""):
    """把普通字符串和 {en, ru} 拼成一条 {en, ru}。"""
    return T(sep.join(_en(p) for p in parts), sep.join(_ru(p) for p in parts))


def err2(e):
    """异常的两种语言文字：feetech.tr_error() 抛的带 .ru，别的异常两边一样。"""
    return T(f"{type(e).__name__}: {e}", f"{type(e).__name__}: {getattr(e, 'ru', e)}")


def release_speed(ids):
    """把速度上限放开、加速度设最大（41 = 0，46 = MAX_SPEED_REG）。只写这两个，**不碰目标位置 42**：
    要是一包写 41..47 连目标一起、目标取自回读的 67，校准收尾刚把目标对准当前读数而
    67 还没跟上，就会把旧目标写回去，下次开扭矩关节被拉走。"""
    ids = list(ids)
    if ids:
        BUS.sync_write(41, 1, [(i, bytes([0])) for i in ids])
        BUS.sync_write(46, 2, [(i, struct.pack("<H", enc_sm(MAX_SPEED_REG))) for i in ids])


def read_phase():
    """连接时只读相位 18，确定速度单位；保留舵机现有的速度、加速度和目标。"""
    global SPEED_UNIT, MAX_SPEED_REG
    if is_fake() or not PRESENT:
        return
    try:
        ph = BUS.read_u8(PRESENT[0], 18)
        SPEED_UNIT = 1.0 if ph & 0x04 else 50.0
        MAX_SPEED_REG = 0 if ph & 0x08 else int(3000 / SPEED_UNIT)
        log(T(f"Phase 18 = {ph} (BIT2={'1' if ph & 4 else '0'} → speed unit {SPEED_UNIT:g} counts/s; BIT3={'1: speed 0 = maximum' if ph & 8 else '0: speed 0 = stop'} → 'no limit' is written as {MAX_SPEED_REG})",
              f"Фаза 18 = {ph} (BIT2={'1' if ph & 4 else '0'} → единица скорости {SPEED_UNIT:g} отсч./с; BIT3={'1: скорость 0 = максимум' if ph & 8 else '0: скорость 0 = стоп'} → без ограничения пишем {MAX_SPEED_REG})"))
    except Exception as e:
        ee = err2(e)
        log(T(f"Phase not read, speed unit assumed 1 count/s: {ee['en']}", f"Фаза не прочиталась, единица скорости принята 1 отсч./с: {ee['ru']}"), "system", "warn", exc=True)


def list_ports():
    """系统里现在有哪些串口。拔插 USB 后口号会变，页面上直接选，不用重启服务。"""
    try:
        from serial.tools import list_ports as lp
        return [{"device": p.device, "desc": (p.description or "").strip()} for p in lp.comports()]
    except Exception as e:
        ee = err2(e)
        log(T(f"Port list not available: {ee['en']}", f"Список портов не получен: {ee['ru']}"), "bus", "warn")
        return []


def bus_trace(line):
    """串口收发的原始字节，只进日志文件（页面不显示，不然刷屏）。
    出问题时 grep 日志文件里的 → / ← 就能还原当时总线上到底发生了什么。"""
    log(line, "bus", "debug")


def open_bus(port, baud=None):
    """打开串口并接管总线。失败不抛异常，返回 {ok, msg}，页面照样能用（假总线）。"""
    global BUS, PORT, BAUD, PRESENT
    baud = baud or BAUD
    with CALIB_LOCK:   # 换口时轮询停下，不然正好读到刚关掉的口
        old, same = BUS, port == PORT
        if same:
            # Windows 串口独占：同一个口要先关自己手里的旧句柄（拔插后它已失效），否则必定 Access denied
            try:
                with old._io:
                    old.close()
            except Exception:
                pass
        try:
            bus = feetech.FeetechBus(port, baud)
        except Exception as e:
            ports = ", ".join(p["device"] for p in list_ports())
            ee = err2(e)
            msg = T(f"Cannot open {port}: {ee['en']}; ports in the system: {ports or 'none'}",
                    f"Не открывается {port}: {ee['ru']}; порты в системе: {ports or 'ни одного'}")
            log(msg, "bus", "error")
            if same:
                fallback_fake(T(f"Port {port} lost", f"Порт {port} потерян"))
            return {"ok": False, "msg": msg}
        bus.trace = bus_trace
        BUS, PORT, BAUD = bus, port, baud
        if not same and old is not None and not isinstance(old, FakeBus):
            try:
                old.close()
            except Exception:
                pass
    log(T(f"Port {port} @ {baud} opened", f"Порт {port} @ {baud} открыт"), "bus")
    do_scan(IDS)
    read_phase()
    return {"ok": True, "msg": T(f"{port} connected, servos on the line: {len(PRESENT)}", f"{port} подключён, на линии серв: {len(PRESENT)}")}


def fallback_fake(why):
    """没有串口时退到假总线：页面照开，能看界面、看 3D，就是不动真舵机。"""
    global BUS, PORT, PRESENT
    BUS, PORT = FakeBus(IDS), None
    PRESENT = list(IDS)
    log(T(f"{_en(why)} — the page is running on the fake bus; after plugging in USB pick the port at the top of the page and press “Connect”",
          f"{_ru(why)} — страница поднята на фиктивной шине; после подключения USB выбрать порт вверху страницы и нажать «Подключить»"), "bus", "warn")


def log_path():
    return os.path.join(LOG_DIR, f"servo-web-{time.strftime('%Y-%m-%d')}.log")


def log(msg, cat="system", level="info", exc=False):
    """msg：字符串或 T(en, ru)。cat：system / bus / motion / calibration / registers / direction。level：info / warn / error / debug。
    每行都追加到 logs/servo-web-日期.log（debug 只进文件，不上页面）；exc=True 把当前异常的 traceback 一起写进文件。
    出问题时这个文件就是现场。"""
    global LOG_N
    t = time.strftime("%H:%M:%S")
    line = f"{t} [{cat}]{LEVEL_TAG.get(level, '')} {_en(msg)}"
    tb = traceback.format_exc() if exc else ""
    with _log_io:
        try:
            os.makedirs(LOG_DIR, exist_ok=True)
            with open(log_path(), "a", encoding="utf-8") as f:
                f.write(line + "\n" + tb)
        except OSError:
            pass
        if level != "debug":
            LOG_N += 1
            LOG.append({"n": LOG_N, "t": t, "cat": cat, "level": level, "msg": T(_en(msg), _ru(msg))})
            del LOG[:-400]
    print(line + ("\n" + tb.rstrip() if tb else ""), flush=True)
    return line


def _ack(err):
    """写指令的应答：0 = 正常；None = 没回包（应答级别 0，或者丢了）；别的 = 舵机状态位有报警。"""
    return T("✓", "✓") if err == 0 else (T("(no reply)", "(нет ответа)") if err is None else T(f"(status 0x{err:02X})", f"(статус 0x{err:02X})"))


def fmt_off(v, ru=False):
    """偏移寄存器 31：HD-1910 第 15 位是符号位，34279 = -1511。
    传进来的可能是原始寄存器值，也可能是已经解过符号的负数，两种都得印对。"""
    if v is None:
        return "?"
    if v < 0:
        return str(v)                                  # 已经是带符号的，直接印
    return f"{feetech.sign15(v)} ({'сырое' if ru else 'raw'} {v})" if v & 0x8000 else str(v)


def is_fake():
    return isinstance(BUS, FakeBus)


async def index(request):
    # 不让浏览器缓存页面：改了前端以后，缓存里的旧页面会用旧逻辑跑，现象像"改了没生效"
    return FileResponse(os.path.join(HERE, "index.html"),
                        headers={"Cache-Control": "no-store, max-age=0"})


async def model_file(request):
    name = request.path_params["name"]
    if name not in ("model.json", "meshes.bin"):
        return JSONResponse({"error": "not found"}, status_code=404)
    path = os.path.join(HERE, "model", name)
    if not os.path.exists(path):
        return JSONResponse({"error": T("the model/ folder is missing", "нет папки model/")}, status_code=404)
    return FileResponse(path)


DIRS_FILE = os.path.join(HERE, "directions.json")


def load_dirs():
    """每颗舵机的方向 +1 / -1。文件里没写的按 +1。"""
    try:
        with open(DIRS_FILE, encoding="utf-8") as f:
            raw = json.load(f)
        return {int(k): (-1 if v == -1 else 1) for k, v in raw.items() if not k.startswith("_")}
    except FileNotFoundError:
        return {}
    except Exception as e:
        log(T(f"directions.json unreadable, all directions assumed +1: {e}", f"directions.json не читается, все направления приняты +1: {e}"), "direction", "warn")
        return {}


MODEL_FILE = os.path.join(HERE, "model", "model.json")
K_RAD = 4096 / (2 * math.pi)


def load_limits():
    """Joint limits (rad) from model/model.json (joint range of the export) -> {id: (lo, hi)}."""
    try:
        with open(MODEL_FILE, encoding="utf-8") as f:
            mj = json.load(f)
        return {int(b["joint"]["id"]): (float(b["joint"]["range"][0]), float(b["joint"]["range"][1]))
                for b in mj["bodies"] if b.get("joint") and b["joint"].get("id") is not None}
    except Exception as e:
        log(T(f"model/model.json unreadable — goals are NOT clamped to the joint limits: {e}", f"model/model.json не читается — цели НЕ ограничены пределами суставов: {e}"), "motion", "warn")
        return {}


LIMITS = load_limits()
COUNT_LIMITS = {34: (1693, 2062)}   # beak: measured raw counts (2062 closed, 1693 open), independent of the ± direction


def clamp_goals(goals):
    """Safety net: every servo goal is clamped to the model joint range, taking the ± sign into account.
    Returns (goals, list of clamped entries "id: requested → clamped")."""
    dirs = load_dirs(); out = {}; cut = []
    for i, p in goals.items():
        lim = LIMITS.get(i)
        if lim or i in COUNT_LIMITS:
            d = dirs.get(i, 1)
            a, b = COUNT_LIMITS.get(i) or sorted((2048 + d * lim[0] * K_RAD, 2048 + d * lim[1] * K_RAD))
            q = int(round(min(max(p, a), b)))
            if q != p:
                cut.append(f"{i}: {p} → {q}")
            p = q
        out[i] = p
    if cut:
        log(T("goal beyond the joint limit, clamped: " + ", ".join(cut), "цель за пределом сустава, обрезано: " + ", ".join(cut)), "motion", "warn")
    return out, cut


def save_dirs(dirs):
    note = ""
    try:
        with open(DIRS_FILE, encoding="utf-8") as f:
            note = json.load(f).get("_note", "")
    except Exception:
        pass
    out = {"_note": note} if note else {}
    for i in IDS:
        out[str(i)] = dirs.get(i, 1)
    with open(DIRS_FILE, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=1)


CALIB_DIR = os.path.join(HERE, "calib")
SIM_CALIB_DIR = None   # 模拟模式的校准备份：放临时目录，服务一关就没了


def calib_dir():
    """校准备份放哪。模拟模式不能用真的 calib/：虚拟舵机的偏移要是混进去，
    接上真舵机点「撤销」，会把虚拟的偏移写进真舵机。"""
    global SIM_CALIB_DIR
    if not is_fake():
        return CALIB_DIR
    if SIM_CALIB_DIR is None or not os.path.isdir(SIM_CALIB_DIR):
        SIM_CALIB_DIR = tempfile.mkdtemp(prefix="servo-web-sim-calib-")
    return SIM_CALIB_DIR


def backup_calibration(ids):
    """中位校准前把每颗的偏移（寄存器 31）、读数、扭矩、锁标志、固件版本记下来，撤销用。"""
    rows = {}
    for i in ids:
        try:
            ver = BUS.read(i, 0, 5)[1]
            rows[str(i)] = {"offset": BUS.read_u16(i, 31), "pos": feetech.sign15(BUS.read_u16(i, 56)),
                            "torque": BUS.read_u8(i, 40), "lock": BUS.read_u8(i, 55), "firmware": f"{ver[0]}.{ver[1]}"}
        except Exception as e:
            rows[str(i)] = {"error": f"{type(e).__name__}: {e}"}
            ee = err2(e)
            log(T(f"#{i} unreadable during backup: {ee['en']}", f"#{i} не читается при резервировании: {ee['ru']}"), "calibration", "warn", exc=True)
    d = calib_dir()
    os.makedirs(d, exist_ok=True)
    now = time.time()                                  # 带毫秒：同一秒点两次也不撞名，字典序 = 时间顺序
    fn = f"eeprom-{time.strftime('%Y%m%d-%H%M%S', time.localtime(now))}{int(now * 1000) % 1000:03d}.json"
    with open(os.path.join(d, fn), "w", encoding="utf-8") as f:
        json.dump({"time": time.strftime("%Y-%m-%d %H:%M:%S"), "servos": rows}, f, ensure_ascii=False, indent=1)
    return fn, rows


CALIB_TOLERANCE = 2      # 校准后读数离目标多少步算到位
DROOP_TOLERANCE = 3      # 关扭矩那几毫秒关节垂了多少步以内就不补偿
OFFSET_LIMIT = 2047      # 偏移寄存器 31 的安全量程：手册说 0~2047 表示正、2048~4095 表示负，
                         # 只有 |偏移| ≤ 2047 这一段几种编码理解都一致，超出就不写
CALIB_LOCK = threading.Lock()   # 校准期间不让状态轮询抢总线：一次 sync_read 卡住能占几百毫秒，
                                # 正好落在关扭矩的窗口里，关节就垂下去了
OFFSET_SIGN = None       # +1：读数 = 原始 + 偏移；-1：读数 = 原始 − 偏移。手册没写死，开机探一次
CALIB_BIG_DEG = 30       # 摆好的位置离目标差这么多度以上：页面再确认一次，日志单独列出来（不拦）


def enc_sm(v):
    """符号-幅值编码：寄存器 31（偏移）、42（目标位置）、46（速度）都是 BIT15 当方向位，
    不是二进制补码。写 -5 要写 0x8005；写成 0xFFFB 舵机会理解成 -32763，然后满速撞限位。"""
    v = int(v)
    return (0x8000 | (-v & 0x7FFF)) if v < 0 else (v & 0x7FFF)


enc_off = enc_sm          # 老名字留着，别处还在用


def read_off(i):
    return feetech.sign15(BUS.read_u16(i, 31))


def read_pos(i):
    return feetech.sign15(BUS.read_u16(i, 56))


def write_off(i, v):
    return BUS.write_u16(i, 31, enc_sm(v))


def wait_off_change(i, was, ms=20):
    """写完偏移就轮询它落地没有，最多等 ms 毫秒。比固定 sleep 强在：
    落地通常 1~2 ms，固定睡 20 ms 等于白送 18 ms 的重力下垂。返回 (读到的偏移, 变了没有)。"""
    end = time.monotonic() + ms / 1000.0
    off = was
    while True:
        off = read_off(i)
        if off != was or time.monotonic() >= end:
            return off, off != was


def probe_offset_sign(i):
    """探一次偏移的符号方向：扭矩开着时把偏移 +1，看读数往哪边动。

    只动 1 步（0.09°），而且在校准窗口之外做，不影响精度。同型号固件一致，探一次给 15 颗共用。
    兜底方案靠它一次写到位；探不出来就两个方向都试（慢，但仍然安全）。"""
    global OFFSET_SIGN
    if OFFSET_SIGN is not None:
        return OFFSET_SIGN
    try:
        o0, r0 = read_off(i), read_pos(i)
        if abs(o0 + 1) > OFFSET_LIMIT:
            return None
        BUS.unlock(i)
        write_off(i, o0 + 1)
        wait_off_change(i, o0)
        r1 = read_pos(i)
        write_off(i, o0)                      # 立刻还原，只是探一下
        wait_off_change(i, o0 + 1)
        BUS.lock(i)
        if r1 == r0:
            log(T(f"Offset sign probe: #{i} offset +1 did not change the reading ({r0}); sign unknown, the fallback path tries both",
                  f"Проба знака смещения: #{i} смещение +1 не изменило отсчёт ({r0}); знак неизвестен, в запасном пути пробуем оба"), "calibration", "warn")
            return None
        OFFSET_SIGN = 1 if r1 > r0 else -1
        log(T(f"Offset sign probe: #{i} offset +1 → reading {r0}→{r1}, i.e. “reading = raw {'+' if OFFSET_SIGN > 0 else '−'} offset”",
              f"Проба знака смещения: #{i} смещение +1 → отсчёт {r0}→{r1}, то есть «отсчёт = сырое {'+' if OFFSET_SIGN > 0 else '−'} смещение»"), "calibration")
        return OFFSET_SIGN
    except Exception as e:
        ee = err2(e)
        log(T(f"Offset sign probe failed: {ee['en']}", f"Проба знака смещения не удалась: {ee['ru']}"), "calibration", "warn", exc=True)
        return None


def calibrate_by_offset(i, target, trail):
    """兜底：舵机连 0x0B 也不认时，自己算偏移写寄存器 31。

    符号方向优先用探针的结果（一次写就够）；探不出来就两个方向都试、回读验证。
    只要没成功就把原偏移写回去，中途抛异常也一样 —— 绝不给舵机留一个乱写的偏移。
    返回 (成功与否, 校准后读数, 现在的偏移)。"""
    o0, r0 = read_off(i), read_pos(i)
    signs = [OFFSET_SIGN] if OFFSET_SIGN else [1, -1]
    ok, pos, off = False, r0, o0
    try:
        for sign in signs:
            cand = o0 + sign * (target - r0)
            if abs(cand) > OFFSET_LIMIT:
                trail.append(T(f"offset {cand:+d} out of range, skipped", f"смещение {cand:+d} вне диапазона, пропуск"))
                continue
            write_off(i, cand)
            off, _ = wait_off_change(i, off)
            pos = read_pos(i)
            trail.append(T(f"offset write {cand}→reading {pos}", f"запись смещения {cand}→отсчёт {pos}"))
            if abs(pos - target) <= CALIB_TOLERANCE:
                ok = True
                return True, pos, off
        return False, pos, off
    finally:
        if not ok and off != o0:
            try:
                write_off(i, o0)
                off, _ = wait_off_change(i, off)
                trail.append(T(f"failed, offset restored to {o0}", f"не вышло, смещение возвращено на {o0}"))
            except Exception as e:
                ee = err2(e)
                log(T(f"#{i} the fallback path failed, and restoring the offset failed too: {ee['en']}",
                      f"#{i} запасной путь не удался, и вернуть смещение тоже не удалось: {ee['ru']}"), "calibration", "error", exc=True)


def calibrate_one(i, before, target=2048):
    """一颗舵机的位置校准：让它「现在这个位置」的读数变成 target（默认 2048 = 官方零位）。

    两条路：先用 0x0B 位置校准指令（舵机自己算偏移、自己存 EEPROM），不生效就自己算偏移写寄存器 31。
    HD-1910（固件 3.46）不认扭矩开关写 128：实测解锁、扭矩关着写 128，
    15 颗偏移一个都没变，还照样回"成功"应答 —— 所以**判据不看应答，看偏移寄存器变没变**。

    关扭矩期间关节被重力压下去（实测颈部 0.62 步/毫秒），而 USB 串口一个来回 1~16 ms，窗口没法做到零。
    对策是把下垂**算出来补偿**：

        校准前读数 pre（扭矩还开着，舵机顶着你摆好的位置）
        校准后这个位置读 target（0x0B 的定义），偏移变了 Δ
        下垂 d = pre − target + 符号 × Δ          ← 三个数都读得到

    把偏移再挪 d，你摆好的那个位置就正好读 target。关节本身不动（停在垂下去的地方），
    扭矩恢复后也不会自己回去 —— 要回去用「回零位」按正常限速走。

    顺序：压低加速度/速度 → 解锁 → 读 pre → 关扭矩并**回读确认真关了** → 0x0B → 轮询偏移确认落地
    → 没落地就兜底 → 算下垂补偿 → 目标写成**当前读数**（永远不写 target，判断错了也不会拉关节）
    → 扭矩复原 → 加锁 → 恢复速度。中途出错，finally 里照样写目标、恢复扭矩、加锁。
    返回 {"ok", "pre", "pos", "target", "droop", "method", "why"}。"""
    was_on = before.get("torque") == 1
    old_off = before.get("offset")
    trail, step, ok, pre, pos, off, why = [], T("speed limit", "ограничение скорости"), False, None, None, None, ""
    method, droop = T("0x0B", "0x0B"), 0
    try:
        # 安全网：校准期间把这颗压成慢速小加速度。万一哪一步判断错了，关节也是慢慢走，人能伸手挡住
        BUS.write_u8(i, 41, 10)
        BUS.write_u16(i, 46, enc_sm(200))
        step = T("unlock", "разблокировка")
        trail.append(tj(T("unlock", "разблокировка"), _ack(BUS.unlock(i))))
        step = T("position read", "чтение положения")
        pre = read_pos(i)                     # 扭矩还开着，读到的就是舵机顶着的位置 = 你摆好的位置
        o_before = read_off(i)
        if was_on:
            step = T("torque off", "момент выкл")
            BUS.write_u8(i, 40, 0)
            if BUS.read_u8(i, 40) != 0:       # 这包丢了的话 0x0B 会在扭矩开着时执行，偏移一改就满速拉
                BUS.write_u8(i, 40, 0)
                if BUS.read_u8(i, 40) != 0:
                    raise feetech.tr_error(feetech.BusError, "torque did not switch off (read-back is not 0), stopping here",
                                      "момент не выключился (чтение не 0), дальше не идём")
            trail.append(T("torque off✓", "момент выкл✓"))
        step = T("0x0B position calibration", "0x0B калибровка положения")
        trail.append(tj(f"0x0B→{target}", _ack(BUS.calibrate_to(i, target))))
        step = T("check that the offset changed", "проверка, что смещение изменилось")
        off, changed = wait_off_change(i, o_before)
        pos = read_pos(i)
        ok = changed or abs(pre - target) <= CALIB_TOLERANCE   # 本来就在目标上，偏移不变也算对
        if not ok:
            step = T("fallback: writing the offset to register 31", "запасной путь: запись смещения в регистр 31")
            trail.append(T("0x0B did not change the offset, writing the offset ourselves", "0x0B не изменил смещение, пишем смещение сами"))
            ok, pos, off = calibrate_by_offset(i, target, trail)
            method = T("offset write 31", "запись смещения 31")
        if ok:
            step = T("droop calculation and compensation", "расчёт и компенсация провисания")
            sign = OFFSET_SIGN or 1
            droop = pre - target + sign * (off - o_before)
            if abs(droop) > DROOP_TOLERANCE and abs(off - sign * droop) <= OFFSET_LIMIT:
                write_off(i, off - sign * droop)
                off, _ = wait_off_change(i, off)
                pos = read_pos(i)
                trail.append(T(f"drooped by {droop:+d} counts while torque was off, offset adjusted by {-sign * droop:+d}",
                               f"за время без момента провисло на {droop:+d} отсч., смещение докручено на {-sign * droop:+d}"))
            elif abs(droop) > DROOP_TOLERANCE:
                trail.append(T(f"drooped by {droop:+d} counts, but the compensation is out of range, not applied",
                               f"провисло на {droop:+d} отсч., но компенсация вне диапазона, не применена"))
        step = T("goal write", "запись цели")
        trail.append(tj(T(f"goal {pos}", f"цель {pos}"), _ack(BUS.write_u16(i, 42, enc_sm(pos)))))   # 永远写当前读数
        if was_on:
            step = T("torque on", "момент вкл")
            trail.append(tj(T("torque on", "момент вкл"), _ack(BUS.write_u8(i, 40, 1))))
        if not ok:
            why = T(f"neither 0x0B nor the offset write changed it (still {fmt_off(o_before)}), reading {pos} instead of {target}",
                    f"ни 0x0B, ни запись смещения его не изменили (всё ещё {fmt_off(o_before, True)}), отсчёт {pos} вместо {target}")
    except Exception as e:
        ok = False
        ee = err2(e)
        why = T(f"failure at step “{_en(step)}”: {ee['en']}", f"сбой на шаге «{_ru(step)}»: {ee['ru']}")
        trail.append(tj("✗", why))
        log(tj(f"#{i} ", why), "calibration", "error", exc=True)
    finally:
        # 不管上面怎么错，都要：目标对准当前读数（下次开扭矩不会按旧坐标系把关节拉走）→ 扭矩复原 → 加锁 → 恢复速度
        try:
            p = read_pos(i)
            BUS.write_u16(i, 42, enc_sm(p))
            if was_on and BUS.read_u8(i, 40) != 1:
                BUS.write_u8(i, 40, 1)
                trail.append(T("torque restored", "момент восстановлен"))
        except Exception as e:
            ee = err2(e)
            trail.append(T(f"✗finish (goal write / torque): {ee['en']}", f"✗завершение (запись цели / момент): {ee['ru']}"))
            log(T(f"#{i} finish failed: the servo may be without torque and the goal register holds the old position; write the goal manually before enabling torque: {ee['en']}",
                  f"#{i} завершение не удалось: серва может быть без момента, а в регистре цели старое положение; перед включением момента записать цель вручную: {ee['ru']}"),
                "calibration", "error", exc=True)
        try:
            BUS.lock(i)
            locked = BUS.read_u8(i, 55)
            trail.append(T("lock✓", "блокировка✓") if locked == 1 else T(f"after locking the flag reads {locked}", f"после блокировки флаг читается {locked}"))
        except Exception as e:
            ee = err2(e)
            trail.append(T(f"✗lock: {ee['en']}", f"✗блокировка: {ee['ru']}"))
            log(T(f"#{i} lock failed, the servo may have stayed unlocked: {ee['en']}", f"#{i} блокировка не удалась, серва могла остаться разблокированной: {ee['ru']}"),
                "calibration", "error", exc=True)
        try:
            release_speed([i])
        except Exception:
            pass
    gap = "" if pre is None else f", Δ {(target - pre) * 360 / 4096:+.1f}°"
    head = T(f"#{i} reading {pre if pre is not None else '?'}→{pos if pos is not None else '?'} (target {target}{gap}, {_en(method)})"
             f" offset {fmt_off(old_off)}→{fmt_off(off)}"
             + (f" droop {droop:+d} counts" if droop else "")
             + ("" if before.get("lock") == 1 else f" lock flag before calibration {before.get('lock')}")
             + f" torque was {'on' if was_on else 'off'}",
             f"#{i} отсчёт {pre if pre is not None else '?'}→{pos if pos is not None else '?'} (цель {target}{gap}, {_ru(method)})"
             f" смещение {fmt_off(old_off, True)}→{fmt_off(off, True)}"
             + (f" провисание {droop:+d} отсч." if droop else "")
             + ("" if before.get("lock") == 1 else f" флаг блокировки до калибровки {before.get('lock')}")
             + f" момент был {'вкл' if was_on else 'выкл'}")
    log(tj(head, " · ", tj(*trail, sep=" "), "" if ok else tj(T(" · failure: ", " · неудача: "), why)), "calibration", "info" if ok else "warn")
    return {"ok": ok, "pre": pre, "pos": pos, "target": target, "droop": droop, "method": method, "why": why}


def save_json_atomic(path, obj):
    """先写 .tmp 再改名：json.dump 写一半崩了会留个半截文件，
    而读不了的备份文件只会被静默跳过，等于悄悄丢数据。"""
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, indent=1)
    os.replace(tmp, path)


def calib_targets():
    """校准目标：每颗 2048（官方零位）→ (名字, {id: 目标读数})。页面确认框和真校准用同一份，不各算各的。"""
    return T("zero 2048", "ноль 2048"), {i: 2048 for i in IDS}


def calib_plan(ids):
    """校准前给页面看：每颗现在读多少、要写成多少、差几度。只读，不写舵机。
    差得多（> CALIB_BIG_DEG）不拦：装舵盘时转了半圈就是这样，校准正好把它纠回来；
    页面会再确认一次，提醒排除「方向 ± 设反了」。"""
    name, targets = calib_targets()
    st = BUS.states(ids)
    rows = []
    for i in ids:
        s, t = st.get(i), targets.get(i)
        pos = s["pos"] if s else None
        rows.append({"id": i, "joint": JOINT_NAMES.get(i), "pos": pos, "target": t,
                     "diff_deg": None if pos is None or t is None else round((t - pos) * 360 / 4096, 1)})
    return {"type": "calib_plan", "name": name, "rows": rows, "big_deg": CALIB_BIG_DEG, "fake": is_fake()}


def calibrate_mid_all(ids):
    """（见下）。整段持 CALIB_LOCK：状态轮询和另一个标签页都得让路。"""
    with CALIB_LOCK:
        return _calibrate_mid_all(ids)


def _calibrate_mid_all(ids):
    """把选中舵机「现在的位置」校准成 2048，一颗一颗来：每颗只关几毫秒扭矩，别的舵机照样撑着。
    鸭子要摆成官方零位（腿伸直、脚板垂直、头平视、嘴闭上）。"""
    name, _ = calib_targets()
    targets = {i: 2048 for i in ids}
    sim = is_fake()
    log(T(f"Calibrating {len(ids)} servos to {_en(name)}: {ids}" + (" (simulation: only virtual servos change, backup in a temporary folder)" if sim else ""),
          f"Калибровка {len(ids)} серв в {_ru(name)}: {ids}" + (" (симуляция: меняются только виртуальные сервы, резерв во временной папке)" if sim else "")), "calibration")
    fn, before = backup_calibration(ids)
    log(T(f"Previous offsets, readings, torque and lock flags saved to {'temporary folder ' + calib_dir() + os.sep if sim else 'calib/'}{fn}",
          f"Прежние смещения, отсчёты, момент и флаги блокировки сохранены в {'временной папке ' + calib_dir() + os.sep if sim else 'calib/'}{fn}"), "calibration")
    probe_offset_sign(next((i for i in ids if "offset" in before.get(str(i), {})), ids[0]))
    ok, bad, deltas, big = [], {}, {}, {}
    for i in ids:
        b = before.get(str(i), {})
        if "offset" not in b:
            bad[i] = T("unreadable during backup, servo untouched", "не читалась при резервировании, серва не тронута")
            continue
        r = calibrate_one(i, b, targets[i])
        if r["ok"]:
            ok.append(i)
            # 同一个物理位置（你摆好的那个），读数从 pre 变成了 target —— 下垂已经补偿掉了
            deltas[str(i)] = r["target"] - r["pre"]
            if abs(deltas[str(i)]) * 360 / 4096 > CALIB_BIG_DEG:
                big[i] = deltas[str(i)] * 360 / 4096
        else:
            bad[i] = r["why"]
    bp = os.path.join(calib_dir(), fn)
    with open(bp, encoding="utf-8") as f:
        rec = json.load(f)
    rec.update({"targets": {str(k): v for k, v in targets.items() if k in ids},
                "deltas": deltas, "bad": {str(k): v for k, v in bad.items()}, "sim": sim})
    save_json_atomic(bp, rec)
    if big:
        # 不拦，只标出来：装舵盘时转了半圈（或错了几个齿）会差这么多，校准正好纠回来
        bl = " ".join(f"#{i} {d:+.1f}°" for i, d in big.items())
        log(T(f"For these servos the set position differs from the target by more than {CALIB_BIG_DEG}°: {bl}"
              ". The horn was mounted half a turn (or a few teeth) off: that is fine, calibration fixes exactly that",
              f"У этих серв выставленное положение отличается от цели больше чем на {CALIB_BIG_DEG}°: {bl}"
              ". Качалка посажена с поворотом на полоборота (или на несколько зубьев): нормально, калибровка это и исправляет"), "calibration", "warn")
    log(T(f"Calibration finished ({_en(name)}): succeeded {len(ok)} {ok}" + (f"; failed {len(bad)} {sorted(bad)}, reasons in the lines above" if bad else "")
          + f"; backup {'temporary folder (simulation) ' if sim else 'calib/'}{fn}",
          f"Калибровка завершена ({_ru(name)}): успешно {len(ok)} {ok}" + (f"; неудачно {len(bad)} {sorted(bad)}, причины в строках выше" if bad else "")
          + f"; резерв {'временная папка (симуляция) ' if sim else 'calib/'}{fn}"), "calibration", "warn" if bad or big else "info")
    return {"type": "calib_result", "ok": ok, "bad": {str(k): v for k, v in bad.items()}, "backup": fn,
            "big": {str(k): round(v, 1) for k, v in big.items()}}


def undo_calibration():
    """（见下）。整段持 CALIB_LOCK。"""
    with CALIB_LOCK:
        return _undo_calibration()


def _undo_calibration():
    """把最近一次备份里的偏移写回去（关扭矩 → 解锁 → 写 31 → 加锁），目标位置改成当前读数，扭矩状态照旧。"""
    d = calib_dir()
    files = sorted(f for f in os.listdir(d) if f.startswith("eeprom-")) if os.path.isdir(d) else []
    if not files:
        log(T("Nothing to undo: no calibrations yet" + (" (in simulation only calibrations made in simulation are undone)" if is_fake() else ""),
              "Нечего отменять: калибровок не было" + (" (в симуляции отменяются только калибровки, сделанные в симуляции)" if is_fake() else "")), "calibration", "warn")
        return {"type": "calib_result", "ok": [], "bad": {}, "backup": None}
    fn = files[-1]
    with open(os.path.join(d, fn), encoding="utf-8") as f:
        rec = json.load(f)
    log(T(f"Undo: restoring offsets from calib/{fn} ({rec.get('time', '?')})", f"Отмена: возвращаем смещения из calib/{fn} ({rec.get('time', '?')})"), "calibration")
    ok, bad = [], {}
    for k, row in rec["servos"].items():
        i = int(k)
        if "offset" not in row:
            log(T(f"#{i} the backup has no offset ({row.get('error', '?')}), skipped", f"#{i} в резерве нет смещения ({row.get('error', '?')}), пропуск"), "calibration", "warn")
            continue
        trail, step, on = [], T("state read", "чтение состояния"), False
        try:
            on = BUS.read_u8(i, 40) == 1
            off0, p0 = BUS.read_u16(i, 31), read_pos(i)
            if on:
                step = T("torque off", "момент выкл")
                BUS.write_u8(i, 40, 0)
                if BUS.read_u8(i, 40) != 0:     # 跟校准一样：这包丢了，改偏移就等于让关节满速跑
                    BUS.write_u8(i, 40, 0)
                    if BUS.read_u8(i, 40) != 0:
                        raise feetech.tr_error(feetech.BusError, "torque did not switch off (read-back is not 0), offset not written",
                                          "момент не выключился (чтение не 0), смещение не пишем")
                trail.append(T("torque off✓", "момент выкл✓"))
            step = T("unlock", "разблокировка")
            trail.append(tj(T("unlock", "разблокировка"), _ack(BUS.unlock(i))))
            step = T("offset write", "запись смещения")
            trail.append(tj(T(f"offset write {fmt_off(row['offset'])}", f"запись смещения {fmt_off(row['offset'], True)}"), _ack(BUS.write_u16(i, 31, row["offset"]))))
            step = T("read", "чтение")
            wait_off_change(i, feetech.sign15(off0))   # 轮询落地，别固定睡 20 ms 让关节多垂
            off1, p1 = BUS.read_u16(i, 31), read_pos(i)
            step = T("goal write", "запись цели")
            trail.append(tj(T(f"goal {p1}", f"цель {p1}"), _ack(BUS.write_u16(i, 42, enc_sm(p1)))))
            if on:
                step = T("torque on", "момент вкл")
                trail.append(tj(T("torque on", "момент вкл"), _ack(BUS.write_u8(i, 40, 1))))
            good = off1 == row["offset"]
            (ok.append(i) if good else bad.__setitem__(i, T(f"offset {row['offset']} written, {off1} read back", f"записано смещение {row['offset']}, прочитано {off1}")))
            log(tj(T(f"#{i} offset {fmt_off(off0)}→{fmt_off(off1)} (backup {fmt_off(row['offset'])}) reading {p0}→{p1} · ",
                     f"#{i} смещение {fmt_off(off0, True)}→{fmt_off(off1, True)} (в резерве {fmt_off(row['offset'], True)}) отсчёт {p0}→{p1} · "), tj(*trail, sep=" ")),
                "calibration", "info" if good else "warn")
        except Exception as e:
            ee = err2(e)
            bad[i] = T(f"failure at step “{_en(step)}”: {ee['en']}", f"сбой на шаге «{_ru(step)}»: {ee['ru']}")
            log(T(f"#{i} undo: {bad[i]['en']} (torque {'not restored' if on else 'was off anyway'})",
                  f"#{i} отмена: {bad[i]['ru']} (момент {'не восстановлен' if on else 'и так был выключен'})"), "calibration", "error", exc=True)
        finally:
            try:                                       # 跟校准同样的收尾：目标对准当前读数再恢复扭矩
                p = read_pos(i)
                BUS.write_u16(i, 42, enc_sm(p))
                if on and BUS.read_u8(i, 40) != 1:
                    BUS.write_u8(i, 40, 1)
            except Exception as e:
                ee = err2(e)
                log(T(f"#{i} undo finish failed, the servo may be without torque: {ee['en']}", f"#{i} завершение отмены не удалось, серва может быть без момента: {ee['ru']}"),
                    "calibration", "error", exc=True)
            try:
                BUS.lock(i)
            except Exception as e:
                ee = err2(e)
                log(T(f"#{i} lock failed: {ee['en']}", f"#{i} блокировка не удалась: {ee['ru']}"), "calibration", "error", exc=True)
    os.rename(os.path.join(d, fn), os.path.join(d, "undone-" + fn))
    log(T(f"Undo finished ({fn} renamed to undone-{fn}): offsets restored on {len(ok)} servos" + (f"; failed {sorted(bad)}" if bad else ""),
          f"Отмена завершена ({fn} переименован в undone-{fn}): смещения возвращены у {len(ok)} серв" + (f"; неудачно {sorted(bad)}" if bad else "")),
        "calibration", "warn" if bad else "info")
    return {"type": "calib_result", "ok": ok, "bad": {str(k): v for k, v in bad.items()}, "backup": fn}


async def logfile(request):
    path = log_path()
    if not os.path.exists(path):
        return JSONResponse({"error": T("no journal for today yet", "за сегодня журнала ещё нет")}, status_code=404)
    return FileResponse(path, media_type="text/plain; charset=utf-8", headers={"Cache-Control": "no-store"})


async def info(request):
    return JSONResponse({"version": VERSION, "ids": IDS, "present": PRESENT, "names": JOINT_NAMES, "fake": is_fake(),
                         "stats": BUS.stats, "log": LOG[-50:]})


def torque_on_aligned(ids):
    """开扭矩前先把目标（42）对准当前读数。扭矩关着时用手掰过关节、或者关着时发过目标 / 校准过，
    目标寄存器里还是旧值，一开扭矩舵机就按放开的速度冲过去。
    顺序：读扭矩 → 关着的读位置 → 写目标 = 位置（带应答）→ 直接读 42 核对 → 开扭矩。
    核对读 42 本身，不看状态里的 67：67 在真机上跟不跟得上没验证过。本来就开着的不动它的目标（可能正在走）。
    读不到位置、目标写不进去的那颗不开。返回 (开了的, {id: 目标}, {id: 没开的原因})。"""
    tq = BUS.sync_read(ids, 40, 1)
    already = [i for i in ids if tq.get(i) and tq[i][1][0] == 1]
    need = [i for i in ids if i not in already]
    aligned, bad = {}, {}
    st = BUS.states(need) if need else {}
    for i in need:
        if not st.get(i):
            bad[i] = T("position unreadable, torque not enabled (otherwise the servo would jump to the old goal)", "положение не читается, момент не включён (иначе серва рванёт к старой цели)")
            continue
        p = st[i]["pos"]
        try:
            for _ in range(2):
                BUS.write_u16(i, 42, enc_sm(p))
                got = feetech.sign15(BUS.read_u16(i, 42))
                if got == p:
                    aligned[i] = p
                    break
            else:
                bad[i] = T(f"goal {p} written, {got} read back; torque not enabled", f"записана цель {p}, прочитано {got}; момент не включён")
        except Exception as e:
            ee = err2(e)
            bad[i] = T(f"goal write error ({ee['en']}); torque not enabled", f"ошибка записи цели ({ee['ru']}); момент не включён")
    on = already + [i for i in need if i in aligned]
    if on:
        BUS.sync_write(40, 1, [(i, bytes([1])) for i in on])
    return on, aligned, bad


def do_scan(ids):
    global PRESENT
    found = [i for i in ids if BUS.ping(i) is not None]
    PRESENT = found
    lost = [i for i in ids if i not in found] if len(ids) <= 32 else []
    log(T(f"Scan {ids[0]}..{ids[-1]}: on the line {len(found)} {found}" + (f"; no reply {lost}" if lost else ""),
          f"Сканирование {ids[0]}..{ids[-1]}: на линии {len(found)} {found}" + (f"; нет ответа {lost}" if lost else "")),
        "bus", "warn" if lost else "info")
    return found


def do_timing(n=200):
    """sync_read 全部在线舵机 n 次，看耗时和失败。"""
    ids = PRESENT or IDS
    times, fails = [], 0
    for _ in range(n):
        t = time.perf_counter()
        r = BUS.sync_read(ids, 56, 15)
        times.append((time.perf_counter() - t) * 1000)
        fails += sum(1 for v in r.values() if v is None)
    times.sort()
    res = {"n": n, "ids": len(ids), "min_ms": round(times[0], 2), "avg_ms": round(sum(times) / n, 2),
           "p99_ms": round(times[max(0, int(n * 0.99) - 1)], 2), "max_ms": round(times[-1], 2), "fails": fails,
           "stats": dict(BUS.stats)}
    log(T(f"Timing test: {res}", f"Тест тайминга: {res}"), "bus", "warn" if fails else "info")
    return res


def handle(cmd):
    """WebSocket 指令，同步执行（在线程里跑）。返回要回给前端的 dict 或 None。"""
    op = cmd.get("op")
    if op == "goal":
        g, _ = clamp_goals({int(cmd["id"]): int(cmd["pos"]) & 0xFFFF})
        for i, p in g.items():
            BUS.sync_write(41, 7, [(i, bytes([SLIDER_ACC]) + struct.pack("<H", p) + struct.pack("<H", 0) + struct.pack("<H", SLIDER_SPEED))])
    elif op == "goals":  # {id: pos}
        g, _ = clamp_goals({int(i): int(p) & 0xFFFF for i, p in cmd["goals"].items()})
        items = [(i, struct.pack("<H", p)) for i, p in g.items()]
        BUS.sync_write(42, 2, items)
    elif op == "goals_stream":
        # 连续流：每帧给目标位置，同时把速度上限设成"正好在 dt 内走到"，加速度最大。
        # 舵机就以匹配的速度连续走，不会每帧冲一下停一下。结束时发 stream_end 放开速度。
        dt = max(0.01, float(cmd.get("dt", 0.05)))
        want, _ = clamp_goals({int(i): int(p) & 0xFFFF for i, p in cmd["goals"].items()})
        fresh = [i for i in want if i not in STREAM_LAST]
        if fresh:
            st = BUS.states(fresh)
            for i in fresh:
                STREAM_LAST[i] = (st[i]["pos"] if st.get(i) else want[i])
        items = []
        for i, p in want.items():
            sps = abs(p - STREAM_LAST[i]) / dt * 1.2          # 留 20% 余量，宁可早到一点
            reg = max(3, min(32767, int(round(sps / SPEED_UNIT))))
            items.append((i, bytes([0]) + struct.pack("<H", p) + struct.pack("<H", 0) + struct.pack("<H", reg)))
            STREAM_LAST[i] = p
        BUS.sync_write(41, 7, items)
    elif op == "stream_end":
        ids = [int(i) for i in cmd.get("ids", [])] or list(STREAM_LAST)
        if ids:
            release_speed(ids)
        for i in ids:
            STREAM_LAST.pop(i, None)
    elif op == "goals_profile":
        # 平滑运动：一次写 41..47 = 加速度(1) 目标位置(2) 时间(2)=0 速度(2)，舵机自己走梯形曲线到终点。
        # 速度按 |终点-当前| / 秒数 算，单位由相位 BIT2 定（SPEED_UNIT 步/秒 每单位）。
        # 发完立刻返回，不在这里等——等的话会把后面的指令（比如灵动的头部流）全堵住。页面等完自己发 goals_verify 和 release。
        seconds = max(0.2, float(cmd.get("seconds", 2)))
        acc = max(1, min(254, int(cmd.get("acc", 30))))
        want, _ = clamp_goals({int(i): int(p) & 0xFFFF for i, p in cmd["goals"].items()})
        st = BUS.states(list(want))
        items, speeds = [], {}
        for i, p in want.items():
            cur = st[i]["pos"] if st.get(i) else p
            sps = abs(p - cur) / seconds                       # 步/秒
            reg = max(1, min(32767, int(round(sps / SPEED_UNIT)))) if sps > 0 else 1
            speeds[i] = reg
            items.append((i, bytes([acc]) + struct.pack("<H", p) + struct.pack("<H", 0) + struct.pack("<H", reg)))
        BUS.sync_write(41, 7, items)
        for i in want:
            STREAM_LAST.pop(i, None)
        gl = " ".join(f"{i}:{want[i]}@{speeds[i]}" for i in want)
        log(T(f"Smooth {seconds} s acc={acc} speed unit={SPEED_UNIT:g} counts/s → {gl}", f"Плавно {seconds} с acc={acc} единица скорости={SPEED_UNIT:g} отсч./с → {gl}"), "motion")
    elif op == "release":
        ids = [int(i) for i in cmd.get("ids", [])] or (PRESENT or IDS)
        release_speed(ids)
        for i in ids:
            STREAM_LAST.pop(i, None)
    elif op == "goals_verify":  # 同上，但回读舵机里的目标位置，没收到的重发，最多 4 轮
        want, _ = clamp_goals({int(i): int(p) & 0xFFFF for i, p in cmd["goals"].items()})
        for attempt in range(4):
            BUS.sync_write(42, 2, [(i, struct.pack("<H", p)) for i, p in want.items()])
            time.sleep(0.02)
            st = BUS.states(list(want))
            want = {i: p for i, p in want.items() if not st.get(i) or (st[i]["goal"] & 0xFFFF) != p}
            if not want:
                break
        if want:
            log(T(f"Goal check: {len(want)} servos did not take the goal in 4 attempts {sorted(want)}", f"Проверка цели: у {len(want)} серв цель не записалась за 4 попытки {sorted(want)}"), "motion", "warn")
        return {"type": "goals_verify", "missing": sorted(want), "attempts": attempt + 1}
    elif op == "torque":
        if cmd.get("ids"):
            ids = [int(i) for i in cmd["ids"]]
        else:
            ids = [int(cmd["id"])] if cmd.get("id") is not None else (PRESENT or IDS)
        if int(cmd["on"]):
            on, aligned, bad = torque_on_aligned(ids)
            al = " ".join(f"#{i}→{p}" for i, p in aligned.items())
            log(T(f"Torque on → {on}" + (f"; before enabling, goal = current position: {al}" if aligned else "")
                  + ("; not enabled " + " ".join(f"#{i} ({_en(w)})" for i, w in bad.items()) if bad else ""),
                  f"Момент вкл → {on}" + (f"; перед включением цель = текущее положение: {al}" if aligned else "")
                  + ("; не включён " + " ".join(f"#{i} ({_ru(w)})" for i, w in bad.items()) if bad else "")), "motion", "warn" if bad else "info")
            return {"type": "torque", "on": 1, "ids": on, "aligned": {str(i): p for i, p in aligned.items()},
                    "bad": {str(i): w for i, w in bad.items()}}
        BUS.sync_write(40, 1, [(i, bytes([0])) for i in ids])
        log(T(f"Torque off → {ids}", f"Момент выкл → {ids}"), "motion")
    elif op == "scan":
        return {"type": "scan", "present": do_scan(parse_ids(cmd.get("ids") or DEFAULT_IDS))}
    elif op == "scan_all":
        return {"type": "scan", "present": do_scan(list(range(0, 254)))}
    elif op == "dump":
        sid = int(cmd["id"])
        d = BUS.dump(sid)
        r = d["raw"]
        log(T(f"Dump #{sid}: firmware {r[0]}.{r[1]} servo {r[3]}.{r[4]} END={r[2]} mode={r[33]} phase={r[18]} "
              f"P/D/I={r[21]}/{r[22]}/{r[23]} offset={fmt_off(r[31] | r[32] << 8)} torque={r[40]} lock={r[55]} reply level={r[8]}",
              f"Выгрузка #{sid}: прошивка {r[0]}.{r[1]} серва {r[3]}.{r[4]} END={r[2]} режим={r[33]} фаза={r[18]} "
              f"P/D/I={r[21]}/{r[22]}/{r[23]} смещение={fmt_off(r[31] | r[32] << 8, True)} момент={r[40]} блокировка={r[55]} уровень ответа={r[8]}"), "registers")
        return {"type": "dump", "id": sid, **d}
    elif op == "write_reg":
        sid, addr, size, val = int(cmd["id"]), int(cmd["addr"]), int(cmd.get("size", 1)), int(cmd["value"])
        unlock = bool(cmd.get("unlock"))
        name = feetech.REGISTERS.get(addr, ("?",))[0]
        name_ru = feetech.REG_NAMES_RU.get(addr, name)
        rd = BUS.read_u8 if size == 1 else BUS.read_u16
        old = rd(sid, addr)
        if unlock:
            BUS.unlock(sid)
        try:
            err = (BUS.write_u8 if size == 1 else BUS.write_u16)(sid, addr, val)
        finally:
            if unlock:
                BUS.lock(sid)
        new = rd(sid, addr)
        same = new == val & (0xFF if size == 1 else 0xFFFF)
        ack = _ack(err)
        log(T(f"#{sid} write {addr} “{name}” {old} → {val}{ack['en']}, read back {new}" + (" (with unlock, locked again)" if unlock else "")
              + ("" if same else " (read-back differs from the written value: read-only? out of range? command values such as 128 in the torque register are not meant to read back)"),
              f"#{sid} запись {addr} «{name_ru}» {old} → {val}{ack['ru']}, прочитано {new}" + (" (с разблокировкой, снова заблокирована)" if unlock else "")
              + ("" if same else " (прочитано не то, что записано: только чтение? вне диапазона? командные значения вроде 128 в регистр момента и не должны читаться)")),
            "registers", "info" if same else "warn")
    elif op == "reboot":
        sid = int(cmd["id"])
        BUS.write_u8(sid, 40, 0)
        BUS.reboot(sid)
        log(T(f"#{sid} reboot 0x08 (torque off, back in ~0.8 s)", f"#{sid} перезагрузка 0x08 (момент выключен, вернётся через ~0.8 с)"), "registers")
    elif op == "calibrate":
        return calibrate_mid_all([int(cmd["id"])])
    elif op == "set_id":
        ok = BUS.set_id(int(cmd["old"]), int(cmd["new"]))
        log(T(f"ID change {cmd['old']} → {cmd['new']}: " + ("the new ID answers PING" if ok else "the new ID does not answer PING"),
              f"Смена ID {cmd['old']} → {cmd['new']}: " + ("новый ID отвечает на PING" if ok else "новый ID не отвечает на PING")), "registers", "info" if ok else "error")
        return {"type": "scan", "present": do_scan(IDS)}
    elif op == "timing":
        return {"type": "timing", **do_timing(int(cmd.get("n", 200)))}
    elif op == "log":
        return {"type": "logs", "items": LOG[-100:]}
    elif op == "calib_mid_all":
        ids = [int(i) for i in cmd.get("ids") or []] or list(PRESENT or IDS)
        return calibrate_mid_all([i for i in ids if i in (PRESENT or IDS)])
    elif op == "calib_plan":
        ids = [int(i) for i in cmd.get("ids") or []] or list(PRESENT or IDS)
        return {**calib_plan([i for i in ids if i in (PRESENT or IDS)]),
                "for": cmd.get("for"), "id": cmd.get("id")}
    elif op == "ports":
        return {"type": "ports", "ports": list_ports(), "port": PORT, "fake": is_fake()}
    elif op == "reconnect":
        port = str(cmd.get("port") or PORT or "")
        if not port:
            return {"type": "reconnect", "ok": False, "msg": T("pick a port first", "сначала выбрать порт")}
        r = open_bus(port, int(cmd.get("baud") or BAUD))
        return {"type": "reconnect", **r, "port": PORT, "fake": is_fake(), "present": PRESENT,
                "ports": list_ports()}
    elif op == "calib_undo":
        return undo_calibration()
    elif op == "set_dir":
        dirs = load_dirs()
        sid, d = int(cmd["id"]), (-1 if int(cmd["dir"]) < 0 else 1)
        dirs[sid] = d
        save_dirs(dirs)
        log(T(f"Direction #{sid} → {d:+d} (saved to directions.json)", f"Направление #{sid} → {d:+d} (записано в directions.json)"), "direction")
        return {"type": "dirs", "dirs": dirs}
    elif op == "page_log":
        m = cmd.get("msg", "")
        m = T(str(m.get("en", ""))[:500], str(m.get("ru", m.get("en", "")))[:500]) if isinstance(m, dict) else str(m)[:500]
        log(m, str(cmd.get("cat") or "page"), cmd.get("level") if cmd.get("level") in ("info", "warn", "error") else "info")
    else:
        log(T(f"Unknown command {cmd}", f"Неизвестная команда {cmd}"), "system", "warn")
    return None


async def broadcast(text):
    for ws in list(CLIENTS):
        try:
            await ws.send_text(text)
        except Exception:
            CLIENTS.discard(ws)


def stream_should_poll():
    """这一拍要不要读总线。"""
    return bool(CLIENTS) and bool(PRESENT) and not CALIB_LOCK.locked()


async def stream():
    """按 STREAM_HZ 广播全部舵机状态和新日志。读不到的舵机攒 3 秒报一次，同一个错误 5 秒内只记一次。"""
    sent_n = LOG_N
    miss, miss_t0 = {}, time.monotonic()
    err_last, err_t = "", 0.0
    while True:
        t0 = time.monotonic()
        if stream_should_poll():
            # 校准期间不抢总线：一次 sync_read 遇上掉包能占几百毫秒，
            # 正好卡在关扭矩的窗口里，关节就垂下去了
            try:
                st = await asyncio.to_thread(BUS.states, PRESENT)
                for i, v in st.items():
                    if v is None:
                        miss[i] = miss.get(i, 0) + 1
                await broadcast(json.dumps({"type": "state", "t": time.time(), "states": st, "stats": BUS.stats}))
            except Exception as e:
                ee = err2(e)
                text = ee["en"]
                if text != err_last or t0 - err_t > 5:
                    log(T(f"State poll error: {ee['en']}", f"Ошибка опроса состояния: {ee['ru']}"), "bus", "error", exc=True)
                    err_last, err_t = text, t0
        if t0 - miss_t0 >= 3:
            if miss:
                ml = " ".join(f"#{i}×{n}" for i, n in sorted(miss.items()))
                log(T(f"No reply in 3 s: {ml} (poll {STREAM_HZ}/s; total timeouts {BUS.stats.get('timeout')}, CRC errors {BUS.stats.get('bad_checksum')})",
                      f"Нет ответа за 3 с: {ml} (опрос {STREAM_HZ} раз/с; всего таймаутов {BUS.stats.get('timeout')}, ошибок CRC {BUS.stats.get('bad_checksum')})"), "bus", "warn")
            miss, miss_t0 = {}, t0
        if CLIENTS and LOG_N > sent_n:
            items = [e for e in LOG if e["n"] > sent_n]
            if items:
                sent_n = items[-1]["n"]
                await broadcast(json.dumps({"type": "logs", "items": items}))
        hz = FAKE_STREAM_HZ if is_fake() else STREAM_HZ
        await asyncio.sleep(max(0.0, 1.0 / hz - (time.monotonic() - t0)))


# 指令 → 日志分类；QUIET 是拖滑块、流式这种高频指令，不逐条记
OP_CAT = {"goal": "motion", "goals": "motion", "goals_stream": "motion", "stream_end": "motion", "goals_profile": "motion",
          "release": "motion", "goals_verify": "motion", "torque": "motion", "scan": "bus", "scan_all": "bus", "timing": "bus",
          "dump": "registers", "write_reg": "registers", "reboot": "registers", "set_id": "registers", "calibrate": "calibration",
          "calib_mid_all": "calibration", "calib_undo": "calibration", "calib_plan": "calibration", "set_dir": "direction",
          "log": "system", "page_log": "page"}
QUIET = {"goal", "goals", "goals_stream", "stream_end", "log", "release", "page_log"}


async def ws_endpoint(ws):
    await ws.accept()
    await ws.send_text(json.dumps({"type": "hello", "ids": IDS, "present": PRESENT, "names": JOINT_NAMES,
                                   "fake": is_fake(), "logs": LOG[-150:], "cats": LOG_CATS, "dirs": load_dirs(),
                                   "port": PORT, "ports": list_ports(),
                                   "regs": [[a, *feetech.REGISTERS[a], feetech.REG_NAMES_RU.get(a, feetech.REGISTERS[a][0])] for a in sorted(feetech.REGISTERS)],
                                   "version": VERSION}))
    CLIENTS.add(ws)  # hello 必须先于后台广播，前端先收到功能配置
    try:
        while True:
            cmd = json.loads(await ws.receive_text())
            op = cmd.get("op")
            if op not in QUIET:
                log(f"Received {json.dumps(cmd, ensure_ascii=False)[:300]}", OP_CAT.get(op, "system"), "debug")
            try:
                reply = await asyncio.to_thread(handle, cmd)
            except Exception as e:
                ee = err2(e)
                msg = T(f"{op}: error {ee['en']}", f"{op}: ошибка {ee['ru']}")
                log(msg, OP_CAT.get(op, "system"), "error", exc=True)
                reply = {"type": "error", "msg": msg}
            if reply:
                await ws.send_text(json.dumps(reply))
    except WebSocketDisconnect:
        pass
    finally:
        CLIENTS.discard(ws)


@contextlib.asynccontextmanager
async def lifespan(app):
    tasks = [asyncio.create_task(stream())]
    try:
        yield
    finally:
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)


app = Starlette(routes=[
    Route("/", index),
    Route("/api/info", info),
    Route("/api/logfile", logfile),
    Route("/model/{name}", model_file),
    WebSocketRoute("/ws", ws_endpoint),
], lifespan=lifespan)


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description="Web console for Feetech HD-1910 servos: sliders, calibration, 3D duck.")
    ap.add_argument("--port", help="serial port, e.g. COM5 / /dev/ttyUSB0 / /dev/ttyS2")
    ap.add_argument("--baud", type=int, default=1_000_000, help="baud rate (default 1000000)")
    ap.add_argument("--ids", default=DEFAULT_IDS, help="servo IDs to poll, e.g. 20-24,30-34,10-14")
    ap.add_argument("--fake", action="store_true", help="no servos: virtual servos, to look at the interface")
    ap.add_argument("--host", default="0.0.0.0", help="address to listen on (default 0.0.0.0 = all interfaces)")
    ap.add_argument("--http-port", type=int, default=8080, help="web page port (default 8080)")
    return ap.parse_args(argv)


def main(argv=None):
    global BUS, IDS, PRESENT, BAUD
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")   # 控制台不是 UTF-8（Windows 重定向到文件时）也别让 ✓ 之类的字符把 print 弄崩
    a = parse_args(argv)
    IDS = parse_ids(a.ids)
    BAUD = a.baud
    log(T(f"Servo console v{VERSION} started, journal: {log_path()}", f"Пульт серв v{VERSION} запущен, журнал пишется в {log_path()}"), "system")
    if a.fake or not a.port:
        fallback_fake(T("--port not given (or --fake given)", "Не задан --port (или указан --fake)") if not a.port else "--fake")
    elif not open_bus(a.port, a.baud)["ok"]:
        # 串口打不开（USB 没插、口号变了、被别的程序占着）也要把页面起起来，
        # 插好以后在页面顶栏选串口点「连接」就行，不用重启服务
        fallback_fake(T(f"Port {a.port} cannot be opened", f"Порт {a.port} не открывается"))
    print(f"Servo console v{VERSION} · open http://127.0.0.1:{a.http_port} in a browser  (from the local network: use this computer's IP)", flush=True)
    uvicorn.run(app, host=a.host, port=a.http_port, log_level="warning")


if __name__ == "__main__":
    main()
