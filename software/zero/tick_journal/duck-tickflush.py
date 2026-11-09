#!/usr/bin/env python3
"""robotd tick journal from RAM (/tmp/duck/*.jsonl) to SD (/var/lib/duck/ticks/), gzip.
Every 60 s (duck-tickflush.timer) and once more at shutdown (duck-tickflush-final.service); at most 2 GB on SD, the oldest files are deleted.
Each run appends only the new whole lines (a gzip member at the end of the .gz: the file stays readable by zcat/gzip).
A RAM file that is fully copied and unchanged for 120 s (robotd no longer writes it) is deleted, so RAM does not fill up."""
import glob, gzip, json, os, time

SRC, DST, MAX_MB, IDLE_S = "/tmp/duck", "/var/lib/duck/ticks", 2048, 120
STATE = os.path.join(SRC, ".flush.json")


def main():
    os.makedirs(DST, exist_ok=True)
    try:
        state = json.load(open(STATE))
    except (OSError, ValueError):
        state = {}
    boot = open("/proc/sys/kernel/random/boot_id").read()[:8]
    for src in sorted(glob.glob(os.path.join(SRC, "*.jsonl"))):
        name = os.path.basename(src)
        st = state.setdefault(name, {
            "off": 0, "dst": os.path.join(DST, "%s_%s_%s.gz" % (time.strftime("%Y%m%d-%H%M%S"), boot, name))})
        size = os.path.getsize(src)
        if size < st["off"]:
            st["off"] = 0
        if size > st["off"]:
            with open(src, "rb") as f:
                f.seek(st["off"])
                chunk = f.read(size - st["off"])
            end = chunk.rfind(b"\n") + 1
            if end:
                with open(st["dst"], "ab") as out:
                    out.write(gzip.compress(chunk[:end], 6))
                    out.flush()
                    os.fsync(out.fileno())
                st["off"] += end
        if st["off"] == os.path.getsize(src) and time.time() - os.path.getmtime(src) > IDLE_S:
            os.remove(src)
            del state[name]
    with open(STATE + ".new", "w") as f:
        json.dump(state, f)
    os.replace(STATE + ".new", STATE)
    files = sorted(glob.glob(os.path.join(DST, "*.gz")), key=os.path.getmtime)
    while files and sum(os.path.getsize(p) for p in files) > MAX_MB * 1024 * 1024:
        os.remove(files.pop(0))


if __name__ == "__main__":
    main()
