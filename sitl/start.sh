#!/bin/bash
# Boot ArduCopter SITL headless, then bridge it with MAVProxy:
#   - udpin:0.0.0.0:14550  <- the competitor connects here (no auth, by design)
#   - udpout:watcher:14551 -> read-only telemetry tap for the scoring watcher
#   - udpout:viz:14552     -> read-only telemetry tap for the web dashboard
set -euo pipefail

HOME_LOC="${HOME_LOC:--35.363262,149.165237,584,0}"
SPEEDUP="${SPEEDUP:-1}"

cd /opt/ardupilot

echo "[sitl] starting ArduCopter SITL (home=${HOME_LOC}, speedup=${SPEEDUP})"
./Tools/autotest/sim_vehicle.py \
    -v ArduCopter \
    -w \
    --no-rebuild \
    --no-mavproxy \
    --speedup "${SPEEDUP}" \
    --custom-location "${HOME_LOC}" \
    --add-param-file /opt/challenge.parm \
    &
SITL_PID=$!

# Wait for the SITL binary to open its TCP endpoint before bridging.
for _ in $(seq 1 60); do
    if (echo > /dev/tcp/127.0.0.1/5760) 2>/dev/null; then break; fi
    sleep 1
done

echo "[sitl] SITL up on tcp:5760 — starting MAVProxy bridge"
exec mavproxy.py \
    --master tcp:127.0.0.1:5760 \
    --out "udpin:0.0.0.0:14550" \
    --out "udpout:${WATCHER_HOST:-watcher}:14551" \
    --out "udpout:${VIZ_HOST:-viz}:14552" \
    --daemon \
    --streamrate 5
