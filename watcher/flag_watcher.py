#!/usr/bin/env python3
"""Scoring.

Ground truth is taken straight off the simulator on a tap the player cannot
reach or influence. Operator belief arrives from the ground station, which
knows only what crossed the link. Everything interesting lives in the gap
between the two.

The flag exists in this process and nowhere else. A shell in any container the
player can reach — the relay, the dashboard, the simulator — yields nothing.
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

CHALLENGE = os.environ.get("CHALLENGE", "q0").lower()

# Two ways to issue a flag.
#
#   static   every instance serves the same string. Fine for practice, a
#            local range, or a writeup — and useless the moment one team
#            pastes it into a group chat.
#   dynamic  the string is derived per team from a secret the players never
#            see, so a leaked flag identifies who leaked it and submitting
#            someone else's does not score.
FLAG_MODE = os.environ.get("FLAG_MODE", "static").lower()
FLAG_SECRET = os.environ.get("FLAG_SECRET", "")
TEAM_ID = os.environ.get("TEAM_ID", "solo")

# Leet bodies, one per challenge. The dynamic mode appends a per-team token.
FLAG_BODIES = {
    "q0": "n0_h4ndsh4k3_r3qu1r3d_m4vl1nk_1s_w1d3_0p3n",
    "q2": "0n3_p01s0n3d_w4yp01nt_4nd_th3_sky_g03s_qu13t",
    "q3": "gh0st_1n_th3_m4ch1n3_th3y_n3v3r_s4w_1t_l34v3",
}


def issue_flag():
    if FLAG_MODE == "dynamic":
        if not FLAG_SECRET:
            raise SystemExit("[watcher] FLAG_MODE=dynamic requires FLAG_SECRET")
        body = FLAG_BODIES.get(CHALLENGE, CHALLENGE)
        token = hmac.new(
            FLAG_SECRET.encode(), f"{CHALLENGE}:{TEAM_ID}".encode(), hashlib.sha256
        ).hexdigest()[:12]
        return f"THJCC{{{body}_{token}}}"
    env = os.environ.get("FLAG")
    if env:
        return env
    return f"THJCC{{{FLAG_BODIES.get(CHALLENGE, 'unset_fl4g')}}}"
FLAG = issue_flag()

ENDPOINT = os.environ.get("WATCH_ENDPOINT", "udpin:0.0.0.0:14551")
HTTP_PORT = int(os.environ.get("HTTP_PORT", "9000"))

HOME_LAT = float(os.environ.get("HOME_LAT", "-35.363262"))
HOME_LON = float(os.environ.get("HOME_LON", "149.165237"))
TGT_LAT = float(os.environ.get("TARGET_LAT", "-35.363262"))
TGT_LON = float(os.environ.get("TARGET_LON", "149.166537"))
TGT_RADIUS_M = float(os.environ.get("TARGET_RADIUS_M", "20"))
TOUCHDOWN_AGL_M = float(os.environ.get("TOUCHDOWN_AGL_M", "2.0"))
MIN_FLIGHT_AGL_M = float(os.environ.get("MIN_FLIGHT_AGL_M", "10.0"))

# q2: how long the autopilot must stay silent while airborne to count as killed
KILL_SILENCE_S = float(os.environ.get("KILL_SILENCE_S", "5"))
KILL_MIN_AGL_M = float(os.environ.get("KILL_MIN_AGL_M", "5"))
# q3: how long the deception must hold
DECEPTION_HOLD_S = float(os.environ.get("DECEPTION_HOLD_S", "10"))

MISSION = json.loads(os.environ.get("MISSION_JSON", json.dumps([
    [-35.363262, 149.165237],
    [-35.362400, 149.165237],
    [-35.362400, 149.166000],
    [-35.363262, 149.166000],
])))
ENVELOPE_MARGIN_M = float(os.environ.get("ENVELOPE_MARGIN_M", "15"))

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


def in_envelope(lat, lon):
    lats = [p[0] for p in MISSION]
    lons = [p[1] for p in MISSION]
    dlat = ENVELOPE_MARGIN_M / 110540.0
    dlon = ENVELOPE_MARGIN_M / (111320.0 * math.cos(math.radians(lats[0])))
    return (min(lats) - dlat <= lat <= max(lats) + dlat
            and min(lons) - dlon <= lon <= max(lons) + dlon)


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
        self.deception_since = None

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
        "token": SOLVE_TOKEN, "flag": FLAG,
        "challenge": CHALLENGE, "evidence": evidence,
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


def solve(reason, evidence):
    with STATE.lock:
        if STATE.solved:
            return
        STATE.solved = True
    log("=" * 62)
    log(f"SOLVED [{CHALLENGE}] — {reason}: {FLAG}")
    log("=" * 62)
    release_to_dashboard(evidence)


# --------------------------------------------------------------------------
# q0 — rogue landing. One arm/disarm cycle, judged at touchdown.
# --------------------------------------------------------------------------

def score_q0(flight):
    if flight["lat"] is None:
        log("DISARMED but no position fix seen — no score")
        return
    dist = haversine_m(flight["lat"], flight["lon"], TGT_LAT, TGT_LON)
    log(f"DISARMED at ({flight['lat']:.6f}, {flight['lon']:.6f}) "
        f"agl={flight['agl']:.1f}m peak={flight['peak_agl']:.1f}m "
        f"dist_to_LZ={dist:.1f}m")
    if flight["peak_agl"] < MIN_FLIGHT_AGL_M:
        log(f"  reject: never climbed above {MIN_FLIGHT_AGL_M}m")
    elif dist > TGT_RADIUS_M:
        log(f"  reject: touched down {dist:.1f}m from the LZ")
    elif flight["agl"] > TOUCHDOWN_AGL_M:
        log(f"  reject: still {flight['agl']:.1f}m up — that is not a landing")
    else:
        solve("unauthorised landing at the rogue LZ", {
            "lat": round(flight["lat"], 7), "lon": round(flight["lon"], 7),
            "peak_agl_m": round(flight["peak_agl"], 1),
            "dist_to_lz_m": round(dist, 1),
        })


# --------------------------------------------------------------------------
# q2 — autopilot killed in flight. Truth stops arriving while airborne.
# --------------------------------------------------------------------------

def watch_q2():
    silent_since = None
    while True:
        time.sleep(1)
        with STATE.lock:
            t = dict(STATE.truth)
            solved = STATE.solved
        if solved:
            return
        if t["last_msg"] == 0.0:
            continue
        gap = time.time() - t["last_msg"]
        airborne = t["armed"] and t["agl"] >= KILL_MIN_AGL_M
        if gap < 2.0:
            silent_since = None
            continue
        if not airborne:
            continue
        if silent_since is None:
            silent_since = time.time() - gap
            log(f"autopilot silent while airborne at {t['agl']:.1f}m — timing it")
        if time.time() - silent_since >= KILL_SILENCE_S:
            solve("autopilot terminated in flight", {
                "lat": round(t["lat"], 7) if t["lat"] else None,
                "lon": round(t["lon"], 7) if t["lon"] else None,
                "agl_at_loss_m": round(t["agl"], 1),
                "silence_s": round(time.time() - silent_since, 1),
            })
            return


# --------------------------------------------------------------------------
# q3 — deception. Truth at the rogue LZ while the operator sees a normal flight.
# --------------------------------------------------------------------------

def watch_q3():
    while True:
        time.sleep(0.5)
        truth, belief, belief_at = STATE.snapshot()
        with STATE.lock:
            if STATE.solved:
                return
        if truth["lat"] is None or not belief or belief.get("lat") is None:
            continue
        if time.time() - belief_at > 5:
            STATE.deception_since = None
            continue

        truth_at_lz = haversine_m(
            truth["lat"], truth["lon"], TGT_LAT, TGT_LON) <= TGT_RADIUS_M
        belief_nominal = in_envelope(belief["lat"], belief["lon"])
        link_ok = bool(belief.get("link_ok"))

        if truth_at_lz and belief_nominal and link_ok:
            if STATE.deception_since is None:
                STATE.deception_since = time.time()
                log("deception established — operator sees a nominal flight")
            held = time.time() - STATE.deception_since
            if held >= DECEPTION_HOLD_S:
                gap = haversine_m(
                    truth["lat"], truth["lon"], belief["lat"], belief["lon"])
                solve(f"operator deceived for {held:.0f}s", {
                    "truth_lat": round(truth["lat"], 7),
                    "truth_lon": round(truth["lon"], 7),
                    "belief_lat": round(belief["lat"], 7),
                    "belief_lon": round(belief["lon"], 7),
                    "divergence_m": round(gap, 1),
                    "held_s": round(held, 1),
                })
                return
        else:
            if STATE.deception_since is not None:
                log("deception broke")
            STATE.deception_since = None


# --------------------------------------------------------------------------

def truth_thread():
    """Passive tap on the simulator. Never transmits."""
    while True:
        try:
            log(f"binding {ENDPOINT}")
            conn = mavutil.mavlink_connection(ENDPOINT)
            armed = False
            flight = {"lat": None, "lon": None, "agl": 0.0, "peak_agl": 0.0}
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
                    flight.update(lat=lat, lon=lon, agl=agl)
                    if armed:
                        flight["peak_agl"] = max(flight["peak_agl"], agl)
                    with STATE.lock:
                        STATE.truth.update(
                            lat=lat, lon=lon, agl=agl,
                            peak_agl=flight["peak_agl"], last_msg=now,
                        )
                    continue

                now_armed = bool(msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
                with STATE.lock:
                    STATE.truth.update(armed=now_armed, last_msg=now)
                if now_armed and not armed:
                    log("ARMED — flight started")
                    flight.update(peak_agl=0.0)
                    armed = True
                elif armed and not now_armed:
                    armed = False
                    if CHALLENGE == "q0":
                        score_q0(flight)
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
        "challenge": CHALLENGE,
        "solved": STATE.solved,
        "truth": truth,
        "belief": belief,
        "belief_age_s": round(time.time() - belief_at, 1) if belief_at else None,
    })


def main():
    log(f"challenge = {CHALLENGE}")
    log(f"flag mode = {FLAG_MODE}"
        + (f" (team {TEAM_ID})" if FLAG_MODE == "dynamic" else ""))
    log(f"rogue LZ = ({TGT_LAT}, {TGT_LON}) r={TGT_RADIUS_M}m")

    threading.Thread(target=truth_thread, daemon=True).start()
    if CHALLENGE == "q2":
        log(f"scoring: autopilot silent >= {KILL_SILENCE_S}s while above "
            f"{KILL_MIN_AGL_M}m")
        threading.Thread(target=watch_q2, daemon=True).start()
    elif CHALLENGE == "q3":
        log(f"scoring: truth at LZ + operator nominal, held >= {DECEPTION_HOLD_S}s")
        threading.Thread(target=watch_q3, daemon=True).start()
    else:
        log(f"scoring: landing within {TGT_RADIUS_M}m of the LZ after a "
            f">={MIN_FLIGHT_AGL_M}m flight")

    app = web.Application()
    app.router.add_post("/api/belief", belief_handler)
    app.router.add_get("/api/status", status_handler)
    web.run_app(app, host="0.0.0.0", port=HTTP_PORT, print=None)


if __name__ == "__main__":
    main()
