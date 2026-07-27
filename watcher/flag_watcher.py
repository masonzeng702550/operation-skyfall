#!/usr/bin/env python3
"""Scoring.

Ground truth comes off a tap on the simulator that the player cannot reach or
influence. Operator belief comes from the ground station, which knows only what
crossed the link. The objective lives in the gap between the two.

A flag is issued when the two pictures disagree by a real margin while the
operator still reads the link as healthy — which is what sitting in the middle
actually means, and what neither pure command injection nor pure telemetry
forgery can produce on its own. Moving the aircraft is not enough if the
operator watches it move. Faking the downlink is not enough if the aircraft
never leaves its mission.

The flag exists in this process and nowhere else. A shell in any container the
player can reach — the relay, the dashboard, the simulator, the station —
yields nothing.
"""
import hashlib
import hmac
import json
import math
import os
import threading
import time
import urllib.error
import urllib.request

from aiohttp import web
from pymavlink import mavutil

# --- flag issuing ----------------------------------------------------------
#
#   static   every instance serves the same string. Fine for practice, a local
#            range, or a writeup — and useless the moment one team pastes it
#            into a group chat.
#   dynamic  derived per team from a secret the players never see, so a leaked
#            flag names the team that leaked it and submitting someone else's
#            does not score.
FLAG_MODE = os.environ.get("FLAG_MODE", "static").lower()
FLAG_SECRET = os.environ.get("FLAG_SECRET", "")
TEAM_ID = os.environ.get("TEAM_ID", "solo")

FLAG_BODY = "h1j4ck3d_th3_1nb0und_dr0n3_b3f0r3_1t_r34ch3d_0ur_c4mp"


def issue_flag():
    if FLAG_MODE == "dynamic":
        if not FLAG_SECRET:
            raise SystemExit("[watcher] FLAG_MODE=dynamic requires FLAG_SECRET")
        token = hmac.new(
            FLAG_SECRET.encode(), TEAM_ID.encode(), hashlib.sha256
        ).hexdigest()[:12]
        return f"THJCC{{{FLAG_BODY}_{token}}}"
    return os.environ.get("FLAG") or f"THJCC{{{FLAG_BODY}}}"


FLAG = issue_flag()

ENDPOINT = os.environ.get("WATCH_ENDPOINT", "udpin:0.0.0.0:14551")
HTTP_PORT = int(os.environ.get("HTTP_PORT", "9000"))

HOME_LAT = float(os.environ.get("HOME_LAT", "-35.363262"))
HOME_LON = float(os.environ.get("HOME_LON", "149.165237"))

# --- what counts as winning ------------------------------------------------
# The enemy drone is inbound; its mission holds it over its approach, threatening
# the camp. Deceiving the enemy operator is not enough — a frozen telemetry feed
# leaves the REAL aircraft flying its real path straight at us. To score you have
# to actually seize the link and rewrite where the aircraft goes, pulling it off
# its approach and onto our capture point. That takes genuine control of the
# vehicle, not just a lie told to its operator.
CAPTURE_LAT = float(os.environ.get("CAPTURE_LAT", "-35.363262"))
CAPTURE_LON = float(os.environ.get("CAPTURE_LON", "149.166537"))
# The capture point sits well off the enemy's approach pattern, so the aircraft
# only reaches it if someone flies it there — never on its own mission.
CAPTURE_RADIUS_M = float(os.environ.get("CAPTURE_RADIUS_M", "25"))
# It must genuinely be airborne, not spoofed from the ground.
MIN_AGL_M = float(os.environ.get("MIN_AGL_M", "5"))
# And held there — a fly-through is not control, loitering under your command is.
HOLD_S = float(os.environ.get("HOLD_S", "8"))
# Belief older than this means the station has stopped reporting (dashboard only).
BELIEF_STALE_S = float(os.environ.get("BELIEF_STALE_S", "5"))

VIZ_URL = os.environ.get("VIZ_URL", "http://viz:8080/api/solved")
SOLVE_TOKEN = os.environ.get("SOLVE_TOKEN", "change-me")

MAV_MODE_FLAG_SAFETY_ARMED = 128


def log(msg):
    print(f"[watcher] {msg}", flush=True)


def haversine_m(lat1, lon1, lat2, lon2):
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.truth = {
            "lat": None, "lon": None, "agl": 0.0,
            "armed": False, "peak_agl": 0.0, "last_msg": 0.0,
        }
        self.belief = {}
        self.belief_at = 0.0
        self.solved = False
        self.holding_since = None

    def snapshot(self):
        with self.lock:
            return dict(self.truth), dict(self.belief), self.belief_at


STATE = State()


def release_to_dashboard(evidence):
    """Hand the earned flag to the dashboard.

    MAVProxy forwards output->master only, never output->output, so a
    STATUSTEXT injected on this read-only tap could never reach the player.
    """
    body = json.dumps({
        "token": SOLVE_TOKEN, "flag": FLAG, "evidence": evidence,
    })
    req = urllib.request.Request(
        VIZ_URL, data=body.encode(), headers={"Content-Type": "application/json"}
    )
    for attempt in range(5):
        try:
            with urllib.request.urlopen(req, timeout=5) as resp:
                log(f"flag released to dashboard (HTTP {resp.status})")
                return True
        except Exception as exc:
            log(f"dashboard push failed ({exc}) — retry {attempt + 1}/5")
            time.sleep(2)
    log("dashboard unreachable — flag is on this log only")
    return False


def solve(evidence):
    with STATE.lock:
        if STATE.solved:
            return
        STATE.solved = True
    log("=" * 62)
    log(f"SOLVED — the inbound drone was rewritten onto our capture point: {FLAG}")
    log("=" * 62)
    release_to_dashboard(evidence)


def score():
    """Win when the REAL aircraft has been flown to our capture point and held.

    This is deliberately measured against ground truth off the simulator, not
    against what the operator believes. Freezing or forging the downlink moves
    the operator's picture but not the aircraft, so it never brings the vehicle
    here — only actually taking the link and steering does.
    """
    last_reason = None
    while True:
        time.sleep(0.5)
        truth, belief, belief_at = STATE.snapshot()
        with STATE.lock:
            if STATE.solved:
                return

        reason = None
        dist = None
        if truth["lat"] is None:
            reason = "no ground truth yet"
        elif not truth["armed"] or truth["agl"] < MIN_AGL_M:
            reason = f"aircraft is not airborne (agl {truth['agl']:.1f} m)"
        else:
            dist = haversine_m(truth["lat"], truth["lon"],
                               CAPTURE_LAT, CAPTURE_LON)
            if dist > CAPTURE_RADIUS_M:
                reason = (f"aircraft still on its own path "
                          f"({dist:.0f} m from the capture point)")

        if reason:
            if STATE.holding_since is not None:
                log(f"hold broken: {reason}")
            STATE.holding_since = None
            if reason != last_reason:
                last_reason = reason
            continue

        last_reason = None
        if STATE.holding_since is None:
            STATE.holding_since = time.time()
            log(f"aircraft reached the capture point ({dist:.0f} m) — timing the hold")
        held = time.time() - STATE.holding_since
        if held >= HOLD_S:
            solve({
                "truth_lat": round(truth["lat"], 7),
                "truth_lon": round(truth["lon"], 7),
                "dist_to_capture_m": round(dist, 1),
                "held_s": round(held, 1),
            })
            return


def truth_thread():
    """Passive tap on the simulator. Never transmits."""
    while True:
        try:
            log(f"binding {ENDPOINT}")
            conn = mavutil.mavlink_connection(ENDPOINT)
            armed = False
            peak = 0.0
            while True:
                msg = conn.recv_match(
                    type=["HEARTBEAT", "GLOBAL_POSITION_INT"],
                    blocking=True, timeout=5,
                )
                if msg is None:
                    continue
                now = time.time()
                if msg.get_type() == "GLOBAL_POSITION_INT":
                    lat, lon = msg.lat / 1e7, msg.lon / 1e7
                    agl = msg.relative_alt / 1000.0
                    if haversine_m(lat, lon, HOME_LAT, HOME_LON) > 5000:
                        continue  # pre-EKF-origin garbage
                    if armed:
                        peak = max(peak, agl)
                    with STATE.lock:
                        STATE.truth.update(lat=lat, lon=lon, agl=agl,
                                           peak_agl=peak, last_msg=now)
                    continue

                now_armed = bool(msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
                with STATE.lock:
                    STATE.truth.update(armed=now_armed, last_msg=now)
                if now_armed and not armed:
                    log("ARMED — flight started")
                    peak = 0.0
                    armed = True
                elif armed and not now_armed:
                    armed = False
                    log("DISARMED")
        except Exception as exc:
            log(f"truth tap error: {exc} — retrying in 3s")
            time.sleep(3)


async def belief_handler(request):
    try:
        body = await request.json()
    except Exception:
        return web.json_response({"error": "bad json"}, status=400)
    with STATE.lock:
        STATE.belief = body
        STATE.belief_at = time.time()
    return web.json_response({"ok": True})


async def status_handler(request):
    truth, belief, belief_at = STATE.snapshot()
    return web.json_response({
        "solved": STATE.solved,
        "truth": truth,
        "belief": belief,
        "belief_age_s": round(time.time() - belief_at, 1) if belief_at else None,
    })


def main():
    log(f"flag mode = {FLAG_MODE}"
        + (f" (team {TEAM_ID})" if FLAG_MODE == "dynamic" else ""))
    log(f"objective: fly the real aircraft to the capture point "
        f"({CAPTURE_LAT}, {CAPTURE_LON}) within {CAPTURE_RADIUS_M:.0f} m, "
        f"above {MIN_AGL_M:.0f} m, held >= {HOLD_S:.0f} s")

    threading.Thread(target=truth_thread, daemon=True).start()
    threading.Thread(target=score, daemon=True).start()

    app = web.Application()
    app.router.add_post("/api/belief", belief_handler)
    app.router.add_get("/api/status", status_handler)
    web.run_app(app, host="0.0.0.0", port=HTTP_PORT, print=None)


if __name__ == "__main__":
    main()
