#!/usr/bin/env python3
"""The legitimate operator.

Flies an approved survey mission and, once a minute, re-uploads it — normal
housekeeping for a station that cannot be sure the aircraft still holds the
plan. That re-upload is a window: everything in it crosses the link again.

Every second this station publishes what it BELIEVES is happening. It has no
way to check that belief against reality; it only knows what arrived over the
link. Scoring compares this belief against ground truth taken straight off the
simulator, so deceiving the operator is worth exactly as much as moving the
aircraft.

Exactly one thread reads the link. Everything else waits on the state that
reader publishes, because two readers on one socket lose each other's messages
and the mission upload is the first thing to break when they do.
"""
import json
import os
import queue
import threading
import time
import urllib.request

from pymavlink import mavutil

UPLINK = os.environ.get("UPLINK", "udpout:mitm:14570")
MISSION_FILE = os.environ.get("MISSION_FILE", "/opt/mission.txt")
BELIEF_SINKS = [s for s in os.environ.get(
    "BELIEF_SINKS",
    "http://watcher:9000/api/belief,http://viz:8080/api/belief",
).split(",") if s]

REUPLOAD_PERIOD_S = float(os.environ.get("REUPLOAD_PERIOD_S", "60"))
CRUISE_ALT = float(os.environ.get("CRUISE_ALT", "30"))
LINK_TIMEOUT_S = float(os.environ.get("LINK_TIMEOUT_S", "3"))

MAV_MODE_FLAG_SAFETY_ARMED = 128
COPTER_AUTO = 3
COPTER_GUIDED = 4

COPTER_MODES = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
    5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND", 16: "POSHOLD",
    17: "BRAKE", 21: "SMART_RTL",
}

MISSION_Q = queue.Queue()


def log(msg):
    print(f"[gcs] {msg}", flush=True)


class Belief:
    """Everything this station thinks it knows. All of it arrived over the link."""

    def __init__(self):
        self.lock = threading.Lock()
        self.d = {
            "lat": None, "lon": None, "agl": 0.0,
            "mode": "—", "armed": False,
            "link_ok": False, "alerts": [], "mission_seq": -1,
            "last_heartbeat": 0.0,
        }
        self.custom_mode = None
        self.fix_type = 0
        self.sats = 0

    def update(self, **kw):
        with self.lock:
            self.d.update(kw)

    def snapshot(self):
        with self.lock:
            d = dict(self.d)
        d["link_ok"] = (time.time() - d["last_heartbeat"]) < LINK_TIMEOUT_S
        return d


BELIEF = Belief()


def publish_belief():
    """Tell the scoreboard what we think is going on, once a second."""
    while True:
        payload = json.dumps(BELIEF.snapshot()).encode()
        for sink in BELIEF_SINKS:
            try:
                req = urllib.request.Request(
                    sink, data=payload, headers={"Content-Type": "application/json"}
                )
                urllib.request.urlopen(req, timeout=2).close()
            except Exception:
                pass  # a sink being down must never stop us flying
        time.sleep(1)


def reader(m):
    """The only thread that touches the socket."""
    while True:
        try:
            msg = m.recv_match(blocking=True, timeout=5)
        except Exception as exc:
            log(f"link read error: {exc}")
            time.sleep(1)
            continue
        if msg is None:
            continue
        t = msg.get_type()

        if t == "HEARTBEAT":
            BELIEF.custom_mode = msg.custom_mode
            BELIEF.update(
                last_heartbeat=time.time(),
                armed=bool(msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED),
                mode=COPTER_MODES.get(msg.custom_mode, f"MODE_{msg.custom_mode}"),
            )
        elif t == "GLOBAL_POSITION_INT":
            BELIEF.update(
                lat=msg.lat / 1e7, lon=msg.lon / 1e7,
                agl=msg.relative_alt / 1000.0,
            )
        elif t == "GPS_RAW_INT":
            BELIEF.fix_type = msg.fix_type
            BELIEF.sats = msg.satellites_visible
        elif t in ("MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"):
            MISSION_Q.put(msg)
        elif t == "STATUSTEXT":
            text = msg.text if isinstance(msg.text, str) else msg.text.decode()
            if msg.severity <= mavutil.mavlink.MAV_SEVERITY_WARNING:
                snap = BELIEF.snapshot()
                BELIEF.update(alerts=(snap["alerts"] + [text])[-10:])
                log(f"  ALERT: {text}")


def drain_mission_q():
    while not MISSION_Q.empty():
        try:
            MISSION_Q.get_nowait()
        except queue.Empty:
            break


def load_mission(path):
    """Parse QGC WPL 110 into mission items."""
    items = []
    with open(path) as fh:
        first = fh.readline()
        if not first.startswith("QGC WPL"):
            raise ValueError(f"not a QGC WPL file: {first!r}")
        for line in fh:
            line = line.strip()
            if not line:
                continue
            f = line.split("\t") if "\t" in line else line.split()
            items.append({
                "seq": int(f[0]), "frame": int(f[2]), "command": int(f[3]),
                "p1": float(f[4]), "p2": float(f[5]),
                "p3": float(f[6]), "p4": float(f[7]),
                "x": float(f[8]), "y": float(f[9]), "z": float(f[10]),
                "autocontinue": int(f[11]),
            })
    return items


def upload_mission(m, items):
    """Standard MAVLink mission upload. Every item crosses the link in the clear."""
    drain_mission_q()
    m.mav.mission_count_send(m.target_system, m.target_component, len(items))
    deadline = time.time() + 30
    sent = set()
    acked = False
    while time.time() < deadline and len(sent) < len(items):
        try:
            req = MISSION_Q.get(timeout=3)
        except queue.Empty:
            continue
        if req.get_type() == "MISSION_ACK":
            acked = True
            break
        it = items[req.seq]
        m.mav.mission_item_int_send(
            m.target_system, m.target_component, it["seq"], it["frame"],
            it["command"], 0, it["autocontinue"],
            it["p1"], it["p2"], it["p3"], it["p4"],
            int(it["x"] * 1e7), int(it["y"] * 1e7), it["z"],
        )
        sent.add(req.seq)
        BELIEF.update(mission_seq=req.seq)
    if not acked:
        try:
            acked = MISSION_Q.get(timeout=5).get_type() == "MISSION_ACK"
        except queue.Empty:
            pass
    log(f"mission uploaded ({len(sent)}/{len(items)} items, ack={acked})")
    return acked


def send_cmd(m, cmd, *params):
    m.mav.command_long_send(
        m.target_system, m.target_component, cmd, 0,
        *(list(params) + [0] * (7 - len(params)))
    )


def wait_ready(timeout=300):
    log("waiting for a position estimate")
    end = time.time() + timeout
    while time.time() < end:
        if BELIEF.fix_type >= 3 and BELIEF.sats >= 6:
            log(f"  3D fix, {BELIEF.sats} sats")
            return True
        time.sleep(1)
    return False


def set_mode(m, custom_mode, name, timeout=20):
    end = time.time() + timeout
    while time.time() < end:
        m.mav.set_mode_send(
            m.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            custom_mode,
        )
        time.sleep(1)
        if BELIEF.custom_mode == custom_mode:
            log(f"  mode = {name}")
            return True
    return False


def arm(m, timeout=60):
    end = time.time() + timeout
    while time.time() < end:
        send_cmd(m, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        time.sleep(1)
        if BELIEF.snapshot()["armed"]:
            log("  armed")
            return True
    return False


def main():
    threading.Thread(target=publish_belief, daemon=True).start()

    log(f"connecting {UPLINK}")
    m = mavutil.mavlink_connection(UPLINK, source_system=255, source_component=190)
    m.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0
    )
    m.wait_heartbeat()
    log(f"linked to system {m.target_system}")
    BELIEF.update(last_heartbeat=time.time())

    threading.Thread(target=reader, args=(m,), daemon=True).start()

    if not wait_ready():
        log("no position estimate — giving up")
        return 1

    items = load_mission(MISSION_FILE)
    log(f"loaded {len(items)} mission items from {MISSION_FILE}")
    upload_mission(m, items)

    if not set_mode(m, COPTER_GUIDED, "GUIDED"):
        log("could not enter GUIDED")
        return 1
    if not arm(m):
        log("could not arm")
        return 1

    log(f"takeoff to {CRUISE_ALT}m")
    send_cmd(m, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, CRUISE_ALT)
    time.sleep(15)

    if not set_mode(m, COPTER_AUTO, "AUTO"):
        log("could not enter AUTO")
    send_cmd(m, mavutil.mavlink.MAV_CMD_MISSION_START)
    log("mission running")

    # Routine re-upload. Nothing about it is suspicious, and that is the problem.
    while True:
        time.sleep(REUPLOAD_PERIOD_S)
        log("re-uploading mission (routine)")
        try:
            upload_mission(m, items)
        except Exception as exc:
            log(f"re-upload failed: {exc}")


if __name__ == "__main__":
    raise SystemExit(main())
