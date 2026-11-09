#!/usr/bin/env python3
"""Amplifier MAX98357A service: SD (Zero pin 13 = gpiochip3, line 2) is low all the time,
high only while the card duck-i2s3 is playing. The I2S clocks do not stop (overlay: rockchip,always-on),
so SD goes up and down while the clocks run, the order the datasheet asks for (p. 16: do not remove LRCLK
while BCLK runs; shutdown has click suppression).
The state comes from /proc/asound/<card>/pcm0p/sub0/status (line "state: RUNNING"), polled every 20 ms.
gpioset (libgpiod v2) owns the line: the process holds the value while it lives."""
import glob
import re
import signal
import subprocess
import sys
import time

CHIP, LINE = "gpiochip3", "2"
ON_DELAY, OFF_DELAY, POLL = 0.03, 0.3, 0.02   # 20 ms poll: at 5 ms the service takes 22 % CPU


def status_path():
    for p in glob.glob("/proc/asound/card*/id"):
        if open(p).read().strip() == "ducki2s3":
            return p.replace("/id", "/pcm0p/sub0/status")
    return None


class Line:
    def __init__(self):
        self.proc, self.value = None, None
        self.set(0)

    def set(self, v):
        if self.value == v:
            return
        if self.proc:
            self.proc.terminate()
            self.proc.wait()
        self.proc = subprocess.Popen(["gpioset", "-c", CHIP, f"{LINE}={v}"])
        self.value = v


def main():
    path = None
    while not path:
        path = status_path()
        if not path:
            time.sleep(1.0)
    line = Line()
    running_since = stopped_since = None
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    print(f"duck-amp: {path}, SD=0", flush=True)
    try:
        while True:
            try:
                running = re.search(r"^state: RUNNING", open(path).read(), re.M) is not None
            except OSError:
                running = False
            now = time.monotonic()
            if running:
                stopped_since = None
                running_since = running_since or now
                if now - running_since >= ON_DELAY:
                    line.set(1)
            else:
                running_since = None
                stopped_since = stopped_since or now
                if now - stopped_since >= OFF_DELAY:
                    line.set(0)
            time.sleep(POLL)
    finally:
        line.set(0)


if __name__ == "__main__":
    main()
