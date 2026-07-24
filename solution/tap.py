#!/usr/bin/env python3
"""Client for the man-in-the-middle tap.

Attach to the relay's TCP port and every datagram crossing the link is handed
to you. Nothing moves unless you send it back, so a passthrough is the minimum
viable attacker — anything you fail to forward is traffic the two endpoints
never see.

    +--------+------------------+-------------------+
    | dir:1  | length:2 (big-e) | MAVLink datagram  |
    +--------+------------------+-------------------+

    UPLINK   0x00   GCS -> UAV
    DOWNLINK 0x01   UAV -> GCS
"""
import socket
import struct

from pymavlink.dialects.v20 import ardupilotmega as mavlink

UPLINK = 0
DOWNLINK = 1


class Tap:
    def __init__(self, host, port, timeout=None):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.buf = b""
        # One parser per direction: MAVLink sequence numbers are per-stream.
        self.parsers = {
            UPLINK: self._new_parser(),
            DOWNLINK: self._new_parser(),
        }
        self.packers = {
            UPLINK: mavlink.MAVLink(None),
            DOWNLINK: mavlink.MAVLink(None),
        }

    @staticmethod
    def _new_parser():
        p = mavlink.MAVLink(None)
        p.robust_parsing = True
        return p

    def frames(self):
        """Yield (direction, raw_datagram) until the relay hangs up."""
        while True:
            while len(self.buf) < 3:
                chunk = self.sock.recv(65536)
                if not chunk:
                    return
                self.buf += chunk
            direction, length = struct.unpack("!BH", self.buf[:3])
            while len(self.buf) < 3 + length:
                chunk = self.sock.recv(65536)
                if not chunk:
                    return
                self.buf += chunk
            payload = self.buf[3:3 + length]
            self.buf = self.buf[3 + length:]
            yield direction, payload

    def send(self, direction, payload):
        """Forward (or inject) a datagram. Never sending one is a drop."""
        if not payload:
            return
        self.sock.sendall(struct.pack("!BH", direction, len(payload)) + payload)

    def decode(self, direction, data):
        """Parse a datagram into MAVLink messages. May be empty."""
        try:
            return self.parsers[direction].parse_buffer(data) or []
        except Exception:
            return []

    def encode(self, direction, messages):
        """Re-serialise messages, preserving each one's original header."""
        packer = self.packers[direction]
        out = b""
        for m in messages:
            hdr = m.get_header()
            packer.seq = hdr.seq
            packer.srcSystem = hdr.srcSystem
            packer.srcComponent = hdr.srcComponent
            out += m.pack(packer)
        return out

    def build(self, direction, msg, src_system=255, src_component=190, seq=0):
        """Serialise a message we invented ourselves."""
        packer = self.packers[direction]
        packer.seq = seq & 0xFF
        packer.srcSystem = src_system
        packer.srcComponent = src_component
        return msg.pack(packer)

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass
