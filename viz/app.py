#!/usr/bin/env python3
"""Read-only range dashboard.

Two views side by side. The left is what the ground station believes, built
only from what crossed the link. The right is what the simulator is actually
doing, taken from a tap the player cannot reach. When those two pictures stop
agreeing, someone is in the middle.

This service holds no flag. The watcher pushes one here only after deciding
the objective was met.
"""
import asyncio
import json
import math
import os
import threading
import time

from aiohttp import WSCloseCode, web
from pymavlink import mavutil

ENDPOINT = os.environ.get("VIZ_ENDPOINT", "udpin:0.0.0.0:14552")
HTTP_PORT = int(os.environ.get("HTTP_PORT", "8080"))

HOME_LAT = float(os.environ.get("HOME_LAT", "-35.363262"))
HOME_LON = float(os.environ.get("HOME_LON", "149.165237"))
TGT_LAT = float(os.environ.get("TARGET_LAT", "-35.363262"))
TGT_LON = float(os.environ.get("TARGET_LON", "149.166537"))
TGT_RADIUS_M = float(os.environ.get("TARGET_RADIUS_M", "20"))
SOLVE_TOKEN = os.environ.get("SOLVE_TOKEN", "change-me")
DIVERGENCE_M = float(os.environ.get("DIVERGENCE_M", "50"))
ENVELOPE_MARGIN_M = float(os.environ.get("ENVELOPE_MARGIN_M", "15"))

MISSION = json.loads(os.environ.get("MISSION_JSON", json.dumps([
    [-35.363262, 149.165237],
    [-35.362400, 149.165237],
    [-35.362400, 149.166000],
    [-35.363262, 149.166000],
])))

MAV_MODE_FLAG_SAFETY_ARMED = 128

COPTER_MODES = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
    5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT",
    13: "SPORT", 14: "FLIP", 15: "AUTOTUNE", 16: "POSHOLD", 17: "BRAKE",
    18: "THROW", 19: "AVOID_ADSB", 20: "GUIDED_NOGPS", 21: "SMART_RTL",
    22: "FLOWHOLD", 23: "FOLLOW", 24: "ZIGZAG", 25: "SYSTEMID",
    26: "AUTOROTATE", 27: "AUTO_RTL",
}

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")


def haversine_m(lat1, lon1, lat2, lon2):
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def in_envelope(lat, lon):
    """Survey-area bounds plus a margin.

    Home sits exactly on a corner of the mission box, so without the margin a
    few centimetres of GPS noise reads as a breach before the aircraft moves.
    """
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
            "connected": False, "lat": None, "lon": None, "agl": 0.0,
            "hdg": 0.0, "vx": 0.0, "vy": 0.0, "vz": 0.0,
            "armed": False, "mode": "—", "peak_agl": 0.0,
            "dist_home_m": 0.0, "dist_lz_m": 0.0,
            "in_envelope": True, "last_msg": 0.0,
        }
        self.belief = {}
        self.belief_at = 0.0
        self.truth_trail = []
        self.belief_trail = []
        self.flag = None
        self.evidence = None

    def update_truth(self, **kw):
        with self.lock:
            self.truth.update(kw)

    def snapshot(self):
        with self.lock:
            truth = dict(self.truth)
            belief = dict(self.belief)
            payload = {
                "truth": truth,
                "truth_trail": list(self.truth_trail),
                "belief": belief,
                "belief_trail": list(self.belief_trail),
                "belief_age_s": (round(time.time() - self.belief_at, 1)
                                 if self.belief_at else None),
                "flag": self.flag,
                "evidence": self.evidence,
            }
        b_lat, b_lon = belief.get("lat"), belief.get("lon")
        if truth["lat"] is not None and b_lat is not None:
            payload["divergence_m"] = round(
                haversine_m(truth["lat"], truth["lon"], b_lat, b_lon), 1)
        else:
            payload["divergence_m"] = None
        payload["verdict"] = verdict_for(truth, belief, payload)
        return payload


def verdict_for(truth, belief, payload):
    if payload["flag"]:
        return "OBJECTIVE MET"
    if not truth["connected"]:
        return "NO TELEMETRY"
    b_lat = belief.get("lat")
    if (b_lat is not None and truth["lat"] is not None
            and payload["divergence_m"] is not None
            and payload["divergence_m"] >= DIVERGENCE_M
            and belief.get("link_ok")):
        return "OPERATOR DECEIVED"
    if truth["lat"] is not None and not truth["in_envelope"]:
        return "OUTSIDE AUTHORISED ENVELOPE"
    return "NOMINAL"


STATE = State()


def mavlink_thread():
    """Blocking pymavlink loop, isolated from the asyncio server."""
    while True:
        try:
            print(f"[viz] binding {ENDPOINT}", flush=True)
            conn = mavutil.mavlink_connection(ENDPOINT)
            while True:
                msg = conn.recv_match(
                    type=["HEARTBEAT", "GLOBAL_POSITION_INT", "VFR_HUD"],
                    blocking=True, timeout=5,
                )
                if msg is None:
                    if time.time() - STATE.truth["last_msg"] > 10:
                        STATE.update_truth(connected=False)
                    continue

                t = msg.get_type()
                if t == "HEARTBEAT":
                    STATE.update_truth(
                        connected=True, last_msg=time.time(),
                        armed=bool(msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED),
                        mode=COPTER_MODES.get(
                            msg.custom_mode, f"MODE_{msg.custom_mode}"),
                    )
                    continue
                if t == "VFR_HUD":
                    STATE.update_truth(last_msg=time.time())
                    continue

                lat, lon = msg.lat / 1e7, msg.lon / 1e7
                agl = msg.relative_alt / 1000.0
                if haversine_m(lat, lon, HOME_LAT, HOME_LON) > 5000:
                    continue  # pre-EKF-origin garbage
                with STATE.lock:
                    peak = (max(STATE.truth["peak_agl"], agl)
                            if STATE.truth["armed"] else STATE.truth["peak_agl"])
                STATE.update_truth(
                    connected=True, last_msg=time.time(),
                    lat=lat, lon=lon, agl=agl,
                    hdg=msg.hdg / 100.0 if msg.hdg != 65535 else 0.0,
                    vx=msg.vx / 100.0, vy=msg.vy / 100.0, vz=msg.vz / 100.0,
                    dist_home_m=haversine_m(lat, lon, HOME_LAT, HOME_LON),
                    dist_lz_m=haversine_m(lat, lon, TGT_LAT, TGT_LON),
                    in_envelope=in_envelope(lat, lon), peak_agl=peak,
                )
                with STATE.lock:
                    STATE.truth_trail.append([lat, lon])
                    if len(STATE.truth_trail) > 1200:
                        del STATE.truth_trail[:200]
        except Exception as exc:
            print(f"[viz] mavlink loop error: {exc} — retrying in 3s", flush=True)
            time.sleep(3)


CONFIG = {
    "home": [HOME_LAT, HOME_LON],
    "target": [TGT_LAT, TGT_LON],
    "target_radius_m": TGT_RADIUS_M,
    "divergence_m": DIVERGENCE_M,
    "mission": MISSION,
}


async def belief_handler(request):
    """The ground station reports what it thinks is happening."""
    try:
        body = await request.json()
    except Exception:
        return web.json_response({"error": "bad json"}, status=400)
    with STATE.lock:
        STATE.belief = body
        STATE.belief_at = time.time()
        if body.get("lat") is not None:
            STATE.belief_trail.append([body["lat"], body["lon"]])
            if len(STATE.belief_trail) > 1200:
                del STATE.belief_trail[:200]
    return web.json_response({"ok": True})


async def solved_handler(request):
    """The watcher — and only the watcher — hands us the earned flag."""
    try:
        body = await request.json()
    except Exception:
        return web.json_response({"error": "bad json"}, status=400)
    if body.get("token") != SOLVE_TOKEN:
        return web.json_response({"error": "forbidden"}, status=403)
    flag = body.get("flag")
    if not flag:
        return web.json_response({"error": "no flag"}, status=400)
    with STATE.lock:
        STATE.flag = flag
        STATE.evidence = body.get("evidence")
    print(f"[viz] flag released by watcher: {body.get('evidence')}", flush=True)
    return web.json_response({"ok": True})


async def ws_handler(request):
    ws = web.WebSocketResponse(heartbeat=20)
    await ws.prepare(request)
    request.app["sockets"].add(ws)
    try:
        await ws.send_json({"type": "config", **CONFIG})
        async for _ in ws:
            pass
    finally:
        request.app["sockets"].discard(ws)
    return ws


async def broadcaster(app):
    try:
        while True:
            payload = {"type": "telemetry", **STATE.snapshot()}
            for ws in list(app["sockets"]):
                try:
                    await ws.send_json(payload)
                except Exception:
                    app["sockets"].discard(ws)
            await asyncio.sleep(0.2)
    except asyncio.CancelledError:
        pass


async def on_startup(app):
    threading.Thread(target=mavlink_thread, daemon=True).start()
    app["broadcaster"] = asyncio.create_task(broadcaster(app))


async def on_cleanup(app):
    app["broadcaster"].cancel()
    for ws in list(app["sockets"]):
        await ws.close(code=WSCloseCode.GOING_AWAY, message=b"shutdown")


def main():
    app = web.Application()
    app["sockets"] = set()
    app.router.add_get("/ws", ws_handler)
    app.router.add_post("/api/belief", belief_handler)
    app.router.add_post("/api/solved", solved_handler)
    app.router.add_get("/api/config", lambda r: web.json_response(CONFIG))
    app.router.add_get("/api/state", lambda r: web.json_response(STATE.snapshot()))
    app.router.add_static("/static/", STATIC_DIR)
    app.router.add_get(
        "/", lambda r: web.FileResponse(os.path.join(STATIC_DIR, "index.html")))
    app.on_startup.append(on_startup)
    app.on_cleanup.append(on_cleanup)
    print(f"[viz] http://0.0.0.0:{HTTP_PORT}", flush=True)
    web.run_app(app, host="0.0.0.0", port=HTTP_PORT, print=None)


if __name__ == "__main__":
    main()
