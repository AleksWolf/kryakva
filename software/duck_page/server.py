"""Duck page on the Zero: status and control, http://<Zero>:8082. Run: sudo python3 server.py (from this folder).
Servos and IMU come from robotd (robot.subscribe / robot.health); the page never touches the servo bus itself.
ToF: tof_check 5 0; camera: v4l2-ctl.
Buttons: Stand up = robotctl robot enable; joystick = robot.move vx/vyaw while the page sends frames
(no frame for 350 ms: zero; robotd has its own deadman); while the DualSense is connected the joystick is off
(padd drives, two speed commands would fight); Power off = relax + poweroff.
3D model: ./model next to this file.
Load: the robotd stream is read only while the page is open (a /state request within 3 s);
camera and ToF run only when switched on on the page (off by default) and only while the page is open.
Messages to the page are keys (the page translates them) plus an untranslated text tail."""
import json, os, socket, subprocess, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
BENCH = "/home/duck/duck-bench"
SOCK = "/run/duck/robot.sock"
MODEL = os.path.join(HERE, "model")
TOF = "/home/duck/zero/tof_check"
VX_MAX, VYAW_MAX = 0.4, 1.0   # command limits of the walking policy training: lin_vel_x ±0.4 m/s, ang_vel_z ±1.0 rad/s
S = {"state": None, "health": None, "tof": None, "cpu_c": None, "pad": False, "log": [], "errors": {}, "want": {"cam": False, "tof": False}}
VIEWER_S = 3.0   # the page polls /state once a second; no request for 3 s = the page is closed
mv = {"vx": 0.0, "vyaw": 0.0, "t": 0.0}


def viewer():
    return time.time() - S.get("t", 0) < VIEWER_S


def log(key, text=""):
    S["log"] = (S["log"] + [[time.strftime("%H:%M:%S"), key, text]])[-30:]


def robotd_running():
    return subprocess.run(["pgrep", "-x", "robotd"], capture_output=True).returncode == 0


def pad_connected():
    """DualSense in the kernel (the same sign padd uses: a Bluetooth input device)."""
    try:
        return "DualSense Wireless Controller" in open("/proc/bus/input/devices").read()
    except OSError:
        return False


def pad_watch():
    while True:
        p = pad_connected()
        if p != S["pad"]:
            log("pad_on" if p else "pad_off")
            if p:
                mv["vx"], mv["vyaw"], mv["t"] = 0.0, 0.0, 0.0
        S["pad"] = p
        time.sleep(0.5)


def check_robotd():
    if not robotd_running():
        log("robotd_down")


class Rpc:
    """One connection to robotd for commands; answers are read line by line."""
    def __init__(self):
        self.s, self.f, self.n, self.lock = None, None, 0, threading.Lock()

    def call(self, method, params):
        with self.lock:
            for attempt in (0, 1):
                try:
                    if self.s is None:
                        self.s = socket.socket(socket.AF_UNIX)
                        self.s.settimeout(2)
                        self.s.connect(SOCK)
                        self.f = self.s.makefile("rb")
                    self.n += 1
                    self.s.sendall((json.dumps({"jsonrpc": "2.0", "id": self.n, "method": method, "params": params}) + "\n").encode())
                    return json.loads(self.f.readline())
                except (OSError, ValueError) as e:
                    self.s = None
                    if attempt:
                        return {"error": str(e)}


rpc = Rpc()


def subscriber():
    while True:
        if not viewer():
            S["state"] = None
            time.sleep(0.5)
            continue
        try:
            s = socket.socket(socket.AF_UNIX)
            s.settimeout(3)
            s.connect(SOCK)
            s.sendall(b'{"jsonrpc":"2.0","id":1,"method":"robot.subscribe","params":{}}\n')
            f = s.makefile("rb")
            f.readline()
            S["errors"].pop("robotd", None)
            for k, ln in enumerate(f):
                if k % 50:         # the stream is 50 Hz and the page updates once a second: every 50th message is parsed
                    continue
                d = json.loads(ln).get("params")
                if d:
                    d.pop("skeleton", None)
                    d.pop("frames", None)
                    d["rx"] = time.time()
                    S["state"] = d
                if not viewer():
                    break
            s.close()
        except (OSError, ValueError) as e:
            S["errors"]["robotd"] = ["robotd_link", str(e)]
            time.sleep(1)


def health():
    while True:
        r = rpc.call("robot.health", {})
        if r and "result" in r:
            S["health"] = r["result"]
        try:
            S["cpu_c"] = int(open("/sys/class/thermal/thermal_zone0/temp").read()) / 1000
        except OSError:
            pass
        time.sleep(1)


def mover():
    """While the page is open and sends frames (no pad): a command 10 times a second, zero too; silent for 350 ms: zero, then nothing."""
    sent = (0.0, 0.0)
    while True:
        live = time.time() - mv["t"] < 0.35   # an open page sends a frame every 100 ms, a zero one too
        cmd = (mv["vx"], mv["vyaw"]) if live else (0.0, 0.0)
        if live or sent != (0.0, 0.0):        # page closed: no commands, after 2 s robotd relaxes
            rpc.call("robot.move", {"vx": cmd[0], "vy": 0.0, "vyaw": cmd[1]})
            if (cmd == (0.0, 0.0)) != (sent == (0.0, 0.0)):
                log("joy" if cmd != (0.0, 0.0) else "stop0")
            sent = cmd
        time.sleep(0.1)


def feed(name, argv, read):
    """The source process (camera / ToF) lives only while it is switched on and the page is open."""
    while True:
        if not (S["want"][name] and viewer()):
            time.sleep(0.3)
            continue
        p = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)

        def stop_when_unwanted():
            while p.poll() is None:
                if not (S["want"][name] and viewer()):
                    p.terminate()
                time.sleep(0.3)
        threading.Thread(target=stop_when_unwanted, daemon=True).start()
        read(p.stdout)
        p.wait()
        if S["want"][name] and viewer():
            S["errors"][name] = ["restart", ""]
            time.sleep(3)
        else:
            S["errors"].pop(name, None)


def tof_read(out):
    for ln in out:
        if ln.startswith(b"F "):
            S["tof"] = {"mm": [int(x) for x in ln.split()[1:65]], "t": time.time()}
    S["tof"] = None


CAM_W, CAM_H = 640, 480
CAM = {"frame": None}


def cam_read(out):
    size = CAM_W * CAM_H * 3 // 2
    while True:
        raw = out.read(size)
        if len(raw) < size:
            break
        CAM["frame"] = raw
    CAM["frame"] = None


def tof():
    feed("tof", [TOF, "5", "0"], tof_read)


def camera():
    # Manual exposure only without rkaiq: with rkaiq_3A_server running, it drives exposure, gain and white balance.
    manual = ("pgrep -x rkaiq_3A_server >/dev/null || "
              "v4l2-ctl -d /dev/v4l-subdev3 --set-ctrl=exposure=2400,analogue_gain=2816 >/dev/null; ")
    cmd = (manual +
           f"exec v4l2-ctl -d /dev/video0 --set-fmt-video=width={CAM_W},height={CAM_H},pixelformat=NV12 --stream-mmap=4 --stream-to=-")
    feed("cam", ["sh", "-c", cmd], cam_read)


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def reply(self, body, ct="application/json"):
        self.send_response(200)
        self.send_header("Content-Type", ct)
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        u = urlparse(self.path)
        if u.path == "/poweroff":
            self.reply(b'{"ok":true}')
            log("poweroff")
            # The servos are powered from the battery, not from the Zero: without relax they keep torque after poweroff.
            mv["vx"], mv["vyaw"], mv["t"] = 0.0, 0.0, 0.0
            try:
                subprocess.run([BENCH + "/bin/robotctl", "--robot-socket", SOCK, "robot", "relax", "--yes"], timeout=3)
            except Exception:
                pass
            os.system("poweroff")
            return
        if u.path == "/move":
            if S["pad"]:
                return self.reply(b'{"ok":false,"why":"pad"}')
            q = parse_qs(u.query)
            mv["vx"] = max(-VX_MAX, min(VX_MAX, float(q.get("vx", ["0"])[0])))
            mv["vyaw"] = max(-VYAW_MAX, min(VYAW_MAX, float(q.get("vyaw", ["0"])[0])))
            mv["t"] = time.time()
        elif u.path == "/toggle":
            q = parse_qs(u.query)
            name = q.get("what", [""])[0]
            if name in S["want"]:
                S["want"][name] = q.get("on", ["0"])[0] == "1"
                log(name + ("_on" if S["want"][name] else "_off"))
        elif u.path == "/stop_move":
            mv["vx"], mv["vyaw"], mv["t"] = 0.0, 0.0, 0.0
        elif u.path == "/relax":
            mv["vx"], mv["vyaw"], mv["t"] = 0.0, 0.0, 0.0
            r = subprocess.run([BENCH + "/bin/robotctl", "--robot-socket", SOCK, "robot", "relax", "--yes"],
                               capture_output=True, text=True, timeout=10)
            log("relax", (r.stdout + r.stderr).strip()[-200:])
        elif u.path == "/standup":
            def go():
                check_robotd()
                r = subprocess.run([BENCH + "/bin/robotctl", "--robot-socket", SOCK, "robot", "enable"],
                                   capture_output=True, text=True, timeout=15)
                log("standup", (r.stdout + r.stderr).strip()[-200:])
            threading.Thread(target=go, daemon=True).start()
        self.reply(b'{"ok":true}')

    def do_GET(self):
        path = self.path.split("?")[0]
        if path == "/state":
            S["t"] = time.time()
            self.reply(json.dumps(S).encode())
        elif path == "/cam.nv12":
            self.reply(CAM["frame"] or b"", "application/octet-stream")
        elif path.startswith("/model/"):
            try:
                self.reply(open(os.path.join(MODEL, os.path.basename(path)), "rb").read(), "application/octet-stream")
            except OSError:
                self.send_error(404)
        else:
            self.reply(open(os.path.join(HERE, "index.html"), "rb").read(), "text/html; charset=utf-8")


check_robotd()
for fn in (subscriber, health, mover, tof, camera, pad_watch):
    threading.Thread(target=fn, daemon=True).start()
ThreadingHTTPServer(("0.0.0.0", 8082), H).serve_forever()
