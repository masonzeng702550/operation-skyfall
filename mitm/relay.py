#!/usr/bin/env python3
"""The man-in-the-middle position.

Every byte between the ground station and the aircraft passes through here.
While nobody is attached the relay is transparent and the mission flies
normally. Once a player attaches to the tap port, they own the link: each
datagram is handed to them and ONLY what they hand back is forwarded.

That asymmetry is the whole point. A player who attaches and then goes quiet
does not get a free ride — they have severed the link, and the operator will
notice. Staying invisible means forwarding everything you are not tampering
with, at the rate the link expects.

Tap wire format (length-prefixed frames, both directions):

    +--------+------------------+-------------------+
    | dir:1  | length:2 (big-e) | MAVLink datagram  |
    +--------+------------------+-------------------+

    dir 0x00 = GCS -> UAV   (uplink: commands, mission uploads, RTCM)
    dir 0x01 = UAV -> GCS   (downlink: telemetry, acks, status text)

A frame with length 0 is a drop. Frames may be injected at any time; they do
not have to correspond to anything received.
"""
import asyncio
import os
import struct
import sys

from pymavlink.dialects.v20 import ardupilotmega as mavlink

GCS_PORT = int(os.environ.get("GCS_PORT", "14570"))
UAV_HOST = os.environ.get("UAV_HOST", "sitl")
UAV_PORT = int(os.environ.get("UAV_PORT", "14550"))
TAP_PORT = int(os.environ.get("TAP_PORT", "14580"))

DIR_UPLINK = 0
DIR_DOWNLINK = 1

MAX_FRAME = 4096


def log(msg):
    print(f"[mitm] {msg}", flush=True)


class Link:
    """Holds both UDP endpoints and the optional attached player."""

    def __init__(self):
        self.gcs_addr = None          # learned from the first uplink datagram
        self.gcs_transport = None
        self.uav_transport = None
        self.player = None            # asyncio.StreamWriter when attached
        self.stats = {"uplink": 0, "downlink": 0, "tapped": 0, "injected": 0}

    def to_uav(self, data):
        if self.uav_transport:
            self.uav_transport.sendto(data)

    def to_gcs(self, data):
        if self.gcs_transport and self.gcs_addr:
            self.gcs_transport.sendto(data, self.gcs_addr)

    async def offer(self, direction, data):
        """Hand a datagram to the attached player, or forward it directly."""
        if self.player is None:
            if direction == DIR_UPLINK:
                self.to_uav(data)
            else:
                self.to_gcs(data)
            return
        try:
            self.player.write(struct.pack("!BH", direction, len(data)) + data)
            await self.player.drain()
            self.stats["tapped"] += 1
        except Exception:
            # Player vanished mid-write; fall back to transparent forwarding.
            self.player = None
            log("player detached (write failed) — link is transparent again")
            await self.offer(direction, data)


LINK = Link()


class GcsProtocol(asyncio.DatagramProtocol):
    def connection_made(self, transport):
        LINK.gcs_transport = transport

    def datagram_received(self, data, addr):
        if LINK.gcs_addr != addr:
            LINK.gcs_addr = addr
            log(f"ground station at {addr[0]}:{addr[1]}")
        LINK.stats["uplink"] += 1
        asyncio.create_task(LINK.offer(DIR_UPLINK, data))


class UavProtocol(asyncio.DatagramProtocol):
    def connection_made(self, transport):
        LINK.uav_transport = transport

    def datagram_received(self, data, addr):
        LINK.stats["downlink"] += 1
        asyncio.create_task(LINK.offer(DIR_DOWNLINK, data))


async def handle_player(reader, writer):
    peer = writer.get_extra_info("peername")
    if LINK.player is not None:
        writer.write(b"")
        writer.close()
        log(f"refused second tap from {peer} — one attacker at a time")
        return

    LINK.player = writer
    log(f"ATTACKER ATTACHED from {peer[0]}:{peer[1]} — link is now yours")
    try:
        while True:
            header = await reader.readexactly(3)
            direction, length = struct.unpack("!BH", header)
            if length > MAX_FRAME:
                log(f"frame too large ({length}) — dropping attacker")
                break
            payload = await reader.readexactly(length) if length else b""
            if not payload:
                continue  # explicit drop
            LINK.stats["injected"] += 1
            if direction == DIR_UPLINK:
                LINK.to_uav(payload)
            else:
                LINK.to_gcs(payload)
    except (asyncio.IncompleteReadError, ConnectionResetError):
        pass
    except Exception as exc:
        log(f"tap error: {exc}")
    finally:
        if LINK.player is writer:
            LINK.player = None
            log("attacker detached — link is transparent again")
        try:
            writer.close()
        except Exception:
            pass


async def report_stats():
    while True:
        await asyncio.sleep(30)
        s = LINK.stats
        log(f"up={s['uplink']} down={s['downlink']} "
            f"tapped={s['tapped']} injected={s['injected']} "
            f"attacker={'yes' if LINK.player else 'no'}")


async def main():
    loop = asyncio.get_running_loop()

    await loop.create_datagram_endpoint(
        GcsProtocol, local_addr=("0.0.0.0", GCS_PORT)
    )
    log(f"listening for the ground station on udp/{GCS_PORT}")

    await loop.create_datagram_endpoint(
        UavProtocol, remote_addr=(UAV_HOST, UAV_PORT)
    )
    log(f"forwarding to the aircraft at {UAV_HOST}:{UAV_PORT}")

    # The aircraft's udpin output only learns a return address once it has
    # received something, so announce ourselves before any traffic exists.
    LINK.to_uav(mavlink.MAVLink_heartbeat_message(
        mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
        mavlink.MAV_STATE_ACTIVE,
    ).pack(mavlink.MAVLink(None, srcSystem=254, srcComponent=190)))

    server = await asyncio.start_server(handle_player, "0.0.0.0", TAP_PORT)
    log(f"tap open on tcp/{TAP_PORT} — attach here to take the link")

    asyncio.create_task(report_stats())
    async with server:
        await server.serve_forever()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        sys.exit(0)
