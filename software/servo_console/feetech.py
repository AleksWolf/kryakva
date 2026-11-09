# -*- coding: utf-8 -*-
"""飞特 STS/SMS 总线的最小实现：包格式、读写、同步读写、重启。

对照飞特 SCS 通信协议手册和磁编码 STS 内存表手册写的，
不依赖飞特 SDK。字节序按磁编码系列：小端。
"""
import struct
import threading
import time

import serial

PING, READ, WRITE, REG_WRITE, ACTION = 0x01, 0x02, 0x03, 0x04, 0x05
RECOVERY, REBOOT, BACKUP, RESET_TURNS, CALIBRATE = 0x06, 0x08, 0x09, 0x0A, 0x0B
SYNC_READ, SYNC_WRITE = 0x82, 0x83
INSTR_NAMES = {0x01: "PING", 0x02: "READ", 0x03: "WRITE", 0x04: "REG_WRITE", 0x05: "ACTION",
               0x06: "parameter reset", 0x08: "REBOOT", 0x09: "parameter backup", 0x0A: "turn count reset", 0x0B: "position calibration",
               0x82: "SYNC_READ", 0x83: "SYNC_WRITE"}
BROADCAST = 0xFE

# 内存表：地址 -> (名字, 字节数, 区)。给寄存器导出用。区：read-only / EEPROM / SRAM / state / factory
REGISTERS = {
    0: ("firmware, major", 1, "read-only"), 1: ("firmware, minor", 1, "read-only"), 2: ("END byte order (0=LE)", 1, "read-only"),
    3: ("servo, major", 1, "read-only"), 4: ("servo, minor", 1, "read-only"),
    5: ("ID", 1, "EEPROM"), 6: ("baud rate (0=1M)", 1, "EEPROM"), 7: ("reserved (STS: return delay)", 1, "EEPROM"),
    8: ("reply level", 1, "EEPROM"), 9: ("min angle limit", 2, "EEPROM"), 11: ("max angle limit", 2, "EEPROM"),
    13: ("temperature limit", 1, "EEPROM"), 14: ("max voltage", 1, "EEPROM"), 15: ("min voltage", 1, "EEPROM"),
    16: ("max torque", 2, "EEPROM"), 18: ("phase", 1, "EEPROM"), 19: ("unload conditions", 1, "EEPROM"),
    20: ("LED alarm conditions", 1, "EEPROM"), 21: ("position P", 1, "EEPROM"), 22: ("position D", 1, "EEPROM"),
    23: ("position I", 1, "EEPROM"), 24: ("min starting torque", 1, "EEPROM"), 25: ("integral limit", 1, "EEPROM"),
    26: ("dead zone +", 1, "EEPROM"), 27: ("dead zone −", 1, "EEPROM"), 28: ("protection current", 2, "EEPROM"),
    30: ("angle resolution", 1, "EEPROM"), 31: ("position offset", 2, "EEPROM"), 33: ("operating mode", 1, "EEPROM"),
    34: ("holding torque", 1, "EEPROM"), 35: ("protection time", 1, "EEPROM"), 36: ("overload torque", 1, "EEPROM"),
    37: ("speed P", 1, "EEPROM"), 38: ("overcurrent protection time", 1, "EEPROM"), 39: ("speed I", 1, "EEPROM"),
    40: ("torque on/off", 1, "SRAM"), 41: ("acceleration", 1, "SRAM"), 42: ("goal", 2, "SRAM"),
    44: ("open-loop PWM", 2, "SRAM"), 46: ("speed", 2, "SRAM"), 48: ("torque limit", 2, "SRAM"),
    55: ("lock flag", 1, "SRAM"),
    56: ("present position", 2, "state"), 58: ("present speed", 2, "state"), 60: ("load", 2, "state"),
    62: ("voltage (0.1 V)", 1, "state"), 63: ("temperature (°C)", 1, "state"), 64: ("REG_WRITE flag", 1, "state"),
    65: ("status bits", 1, "state"), 66: ("moving flag", 1, "state"), 67: ("goal (read-back)", 2, "state"),
    69: ("current (6.5 mA)", 2, "state"),
    80: ("moving speed threshold", 1, "factory"), 81: ("DTs (ms)", 1, "factory"), 82: ("speed unit factor", 1, "factory"),
    83: ("min speed limit", 1, "factory"), 84: ("max speed limit", 1, "factory"), 85: ("acceleration limit", 1, "factory"),
    86: ("acceleration multiplier", 1, "factory"),
}
# 同一张表的俄文名，页面切到 RU 时显示
REG_NAMES_RU = {
    0: "прошивка, старший", 1: "прошивка, младший", 2: "END порядок байт (0=LE)", 3: "серва, старший", 4: "серва, младший",
    5: "ID", 6: "скорость порта (0=1M)", 7: "резерв (у STS — задержка ответа)", 8: "уровень ответа", 9: "предел угла мин",
    11: "предел угла макс", 13: "предел температуры", 14: "напряжение макс", 15: "напряжение мин", 16: "момент макс",
    18: "фаза", 19: "что отключает серву", 20: "что зажигает LED", 21: "P по положению", 22: "D по положению",
    23: "I по положению", 24: "стартовый момент мин", 25: "предел интеграла", 26: "мёртвая зона +", 27: "мёртвая зона −",
    28: "ток защиты", 30: "разрешение угла", 31: "смещение положения", 33: "режим работы", 34: "момент удержания",
    35: "время защиты", 36: "момент перегрузки", 37: "P по скорости", 38: "время защиты по току", 39: "I по скорости",
    40: "момент вкл/выкл", 41: "ускорение", 42: "цель", 44: "PWM разомкнутый", 46: "скорость", 48: "предел момента",
    55: "флаг блокировки", 56: "текущее положение", 58: "текущая скорость", 60: "нагрузка", 62: "напряжение (0.1 В)",
    63: "температура (°C)", 64: "флаг REG_WRITE", 65: "биты состояния", 66: "флаг движения", 67: "цель (чтение)",
    69: "ток (6.5 мА)", 80: "порог скорости движения", 81: "DTs (мс)", 82: "коэффициент единицы скорости",
    83: "предел скорости мин", 84: "предел скорости макс", 85: "предел ускорения", 86: "множитель ускорения",
}
STATUS_BITS = {0: "voltage", 1: "encoder", 2: "temperature", 3: "current", 5: "load"}
DUMP_END = max(REGISTERS) + 1   # 87：寄存器表最后一个地址 86


def sign15(v):
    """BIT15 是方向位的 16 位量（位置、速度、电流）。"""
    return -(v & 0x7FFF) if v & 0x8000 else v


def sign10(v):
    """BIT10 是方向位（负载）。"""
    return -(v & 0x3FF) if v & 0x400 else v


class BusError(Exception):
    pass


def tr_error(cls, en, ru):
    """异常带两种语言：str(e) 是英文，e.ru 是俄文（server.py 发给页面时两种都带）。"""
    e = cls(en)
    e.ru = ru
    return e


class BusTimeout(BusError):
    """没应答或应答不完整。应答级别 0 时写指令本来就不回包，调用方自己决定算不算错。"""


def hexs(b):
    return " ".join(f"{x:02x}" for x in b)


class FeetechBus:
    def __init__(self, port, baud=1_000_000, timeout=0.02, ser=None):
        # ser: a serial-like object for tests; the real bus uses the serial port
        self.ser = ser if ser is not None else serial.Serial(port, baud, timeout=timeout, write_timeout=0.1)
        # 串口互斥锁。别叫 self.lock：会盖住下面的 lock(sid)「加 EEPROM 锁」方法
        self._io = threading.Lock()
        # 原始收发记录：设一个 trace(文本) 回调就开始记。飞特不支持的操作照样回"成功"，
        # 只有对着十六进制才分得清「真做了」和「装作做了」。10 Hz 的状态轮询不记，不然日志全是它
        self.trace = None
        self.trace_skip = {SYNC_READ, SYNC_WRITE}
        self._last = (None, None)
        self.stats = {"tx": 0, "rx_ok": 0, "timeout": 0, "bad_checksum": 0, "bad_id": 0}

    def close(self):
        self.ser.close()

    # ---- 包 ----
    @staticmethod
    def _packet(sid, instr, params=b""):
        length = len(params) + 2
        chk = (~(sid + length + instr + sum(params))) & 0xFF
        return bytes([0xFF, 0xFF, sid, length, instr]) + bytes(params) + bytes([chk])

    def _send(self, sid, instr, params=b""):
        self.ser.reset_input_buffer()
        pkt = self._packet(sid, instr, params)
        self.ser.write(pkt)
        self.stats["tx"] += 1
        self._last = (sid, instr)
        if self.trace and instr not in self.trace_skip:
            note = ""
            if instr in (READ, WRITE) and params:
                reg = REGISTERS.get(params[0])
                note = f"  addr {params[0]}" + (f" “{reg[0]}”" if reg else "") + (
                    f" write {list(params[1:])}" if instr == WRITE else f" read {params[1]} bytes")
            self.trace(f"→ #{sid} {INSTR_NAMES.get(instr, hex(instr))} {hexs(pkt)}{note}")

    def _read_status(self, expect_id=None):
        """读一个应答帧，返回 (id, error, params)。"""
        # 找 FF FF
        hdr = b""
        deadline = time.monotonic() + self.ser.timeout * 3
        while time.monotonic() < deadline:
            b = self.ser.read(1)
            if not b:
                break
            hdr = (hdr + b)[-2:]
            if hdr == b"\xff\xff":
                break
        who = f"id {expect_id}" if expect_id is not None else "servo"
        who_ru = f"id {expect_id}" if expect_id is not None else "серва"
        if hdr != b"\xff\xff":
            self.stats["timeout"] += 1
            ms = self.ser.timeout * 3 * 1000
            raise tr_error(BusTimeout, f"{who} does not answer (no header within {ms:.0f} ms)",
                      f"{who_ru} не отвечает (заголовок не пришёл за {ms:.0f} мс)")
        rest = self.ser.read(2)
        if len(rest) < 2:
            self.stats["timeout"] += 1
            raise tr_error(BusTimeout, f"{who}: reply cut off at ID/length (got ff ff {hexs(rest)})",
                      f"{who_ru}: ответ оборван на ID/длине (получено ff ff {hexs(rest)})")
        sid, length = rest[0], rest[1]
        body = self.ser.read(length)
        if len(body) < length:
            self.stats["timeout"] += 1
            raise tr_error(BusTimeout, f"{who}: incomplete reply, length {length}, got {len(body)} (ff ff {hexs(rest + body)})",
                      f"{who_ru}: ответ неполный, длина {length}, получено {len(body)} (ff ff {hexs(rest + body)})")
        err, params, chk = body[0], body[1:-1], body[-1]
        if ((~(sid + length + err + sum(params))) & 0xFF) != chk:
            self.stats["bad_checksum"] += 1
            raise tr_error(BusError, f"{who}: bad checksum (ff ff {hexs(rest + body)})",
                      f"{who_ru}: неверная контрольная сумма (ff ff {hexs(rest + body)})")
        if expect_id is not None and sid != expect_id:
            self.stats["bad_id"] += 1
            raise tr_error(BusError, f"expected a reply from id {expect_id}, got id {sid} (ff ff {hexs(rest + body)})",
                      f"ждали ответ id {expect_id}, пришёл id {sid} (ff ff {hexs(rest + body)})")
        self.stats["rx_ok"] += 1
        if self.trace and self._last[1] not in self.trace_skip:
            hdr = b"\xff\xff"   # Python 3.11: backslash inside an f-string expression is not allowed
            self.trace(f"← #{sid} status 0x{err:02X} {hexs(hdr + rest + body)}"
                       + (f"  data {list(params)}" if params else ""))
        return sid, err, bytes(params)

    # ---- 指令 ----
    def ping(self, sid):
        with self._io:
            self._send(sid, PING)
            try:
                _, err, _ = self._read_status(sid)
                return err
            except BusError:
                return None

    def read(self, sid, addr, n):
        with self._io:
            self._send(sid, READ, bytes([addr, n]))
            _, err, params = self._read_status(sid)
            if len(params) != n:
                raise tr_error(BusError, f"id {sid}: read of {n} bytes from address {addr}, got only {len(params)}",
                          f"id {sid}: чтение {n} байт с адреса {addr}, пришло только {len(params)}")
            return err, params

    def write(self, sid, addr, data, wait=True):
        with self._io:
            self._send(sid, WRITE, bytes([addr]) + bytes(data))
            if not wait or sid == BROADCAST:
                return None
            try:
                _, err, _ = self._read_status(sid)
                return err
            except BusTimeout:
                # 应答级别 0 时写指令不回包，不算错；调用方拿到 None 自己判断
                return None

    def write_u8(self, sid, addr, v):
        return self.write(sid, addr, [v & 0xFF])

    def write_u16(self, sid, addr, v):
        return self.write(sid, addr, struct.pack("<H", v & 0xFFFF))

    def read_u8(self, sid, addr):
        return self.read(sid, addr, 1)[1][0]

    def read_u16(self, sid, addr):
        return struct.unpack("<H", self.read(sid, addr, 2)[1])[0]

    def sync_write(self, addr, length, id_values):
        """id_values: [(id, bytes), ...]，每个 bytes 长度 = length。"""
        params = bytes([addr, length])
        for sid, data in id_values:
            assert len(data) == length
            params += bytes([sid]) + bytes(data)
        with self._io:
            self._send(BROADCAST, SYNC_WRITE, params)

    def sync_read(self, ids, addr, n):
        """返回 {id: (err, params) 或 None}；按应答里的 ID 归档，缺包不连带丢后续包。"""
        out = dict.fromkeys(ids)
        with self._io:
            self._send(BROADCAST, SYNC_READ, bytes([addr, n]) + bytes(ids))
            for _ in ids:
                try:
                    sid, err, params = self._read_status()
                except BusTimeout:
                    break
                except BusError:
                    continue
                if sid not in out:
                    self.stats["bad_id"] += 1
                elif len(params) == n and out[sid] is None:
                    out[sid] = (err, params)
        return out

    def reboot(self, sid):
        """0x08，无应答，约 800 ms 后回来。"""
        with self._io:
            self._send(sid, REBOOT)

    def calibrate_to(self, sid, value=None):
        """0x0B 位置校准：把舵机「现在这个位置」的读数改成 value（不给就是中位 2048），偏移舵机自己算、自己存。
        协议手册 4.9：STS ≥3.10、HLS ≥3.43 支持带参数。HD-1910（固件 3.46）不认扭矩开关写 128，只能走这个。
        返回应答里的状态字节；没应答返回 None。"""
        params = b"" if value is None else struct.pack("<H", value & 0xFFFF)
        with self._io:
            self._send(sid, CALIBRATE, params)
            try:
                return self._read_status(sid)[1]
            except BusTimeout:
                return None

    def unlock(self, sid):
        return self.write_u8(sid, 55, 0)

    def lock(self, sid):
        return self.write_u8(sid, 55, 1)

    # ---- 组合动作 ----
    def state(self, sid):
        """56..70：位置 速度 负载 电压 温度 异步写 状态 移动 目标 电流 = 15 字节。"""
        err, p = self.read(sid, 56, 15)
        return self._decode_state(err, p)

    @staticmethod
    def _decode_state(err, p):
        pos, spd, load = struct.unpack_from("<HHH", p, 0)
        goal, cur = struct.unpack_from("<HH", p, 11)
        return {
            "err": err, "pos": sign15(pos), "speed": sign15(spd), "load": sign10(load),
            "volt": p[6] / 10.0, "temp": p[7], "status": p[9], "moving": p[10],
            "goal": sign15(goal), "current_ma": sign15(cur) * 6.5,
            "status_text": ", ".join(n for b, n in STATUS_BITS.items() if p[9] >> b & 1) or "",
        }

    def states(self, ids):
        """一次 sync_read 拿全部，失败的 ID 值为 None。"""
        raw = self.sync_read(ids, 56, 15)
        return {sid: (self._decode_state(*v) if v else None) for sid, v in raw.items()}

    def dump(self, sid):
        """0–86 全读，按 REGISTERS 解码。"""
        raw = b""
        for start in range(0, DUMP_END, 10):          # HD-1910 的表到 86 为止，读过头舵机只回剩下的字节
            n = min(10, DUMP_END - start)
            for attempt in range(4):
                try:
                    raw += self.read(sid, start, n)[1]
                    break
                except BusError as e:
                    if attempt == 3:
                        raise tr_error(BusError, f"dump stopped at address {start}: {e}",
                                  f"выгрузка остановилась на адресе {start}: {getattr(e, 'ru', e)}")
                    time.sleep(0.01)
        rows = []
        for addr in sorted(REGISTERS):
            name, size, area = REGISTERS[addr]
            v = raw[addr] if size == 1 else struct.unpack_from("<H", raw, addr)[0]
            rows.append({"addr": addr, "name": name, "area": area, "size": size, "value": v})
        return {"rows": rows, "raw": list(raw)}

    def set_id(self, old, new):
        self.unlock(old)
        # 改 ID 这一包的应答可能用新 ID 回，也可能用旧 ID 回，不等它；下一包发之前会清接收缓冲
        self.write(old, 5, [new & 0xFF], wait=False)
        time.sleep(0.02)
        self.lock(new)
        return self.ping(new) is not None
