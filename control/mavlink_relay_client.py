#!/usr/bin/env python3
"""
MAVLink relay client — local bridge to the mTLS relay on Frank's PC.

Used on TWO different machines with different --role values:
  Jetson       --role vehicle     (mavros's gcs_url connects to this locally)
  MP machine   --role controller  (Mission Planner's TCP connection profile
                                    connects to this locally)

Presents a plain, unencrypted local TCP server (default 127.0.0.1:5760) so
mavros/Mission Planner don't need to know anything about TLS or certs. Dials
out to the relay server over mutual TLS, authenticating itself with the
role's client cert (see streaming/generate_relay_certs.sh) — the relay
decides vehicle vs. controller routing from the cert's CN, not anything this
script claims about itself.

Deliberately blocks rather than drops on backpressure: unlike a live video
stream (where a stale frame is worthless and should be discarded, see
ground_view_stream.py), a dropped MAVLink command is a silent failure — if
the relay leg is down, this script simply doesn't read from the local
socket, so mavros's/MP's own TCP stack sees normal backpressure instead of
losing data.

Usage:
  python3 control/mavlink_relay_client.py --role vehicle
  python3 control/mavlink_relay_client.py --role controller --relay-host 118.232.160.227

Requires streaming/relay_certs/{ca.crt,<role>.crt,<role>.key} — generate once
with streaming/generate_relay_certs.sh and copy to this machine.
"""

import argparse
import os
import socket
import ssl
import sys
import threading
import time

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_CERT_DIR = os.path.join(PROJECT_DIR, "streaming", "relay_certs")

RECONNECT_BACKOFF_START = 1.0
RECONNECT_BACKOFF_MAX = 30.0


def _log(msg: str) -> None:
    print(f"[mavlink_relay:{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def _build_ssl_context(cert_dir: str, role: str) -> ssl.SSLContext:
    ca_file   = os.path.join(cert_dir, "ca.crt")
    cert_file = os.path.join(cert_dir, f"{role}.crt")
    key_file  = os.path.join(cert_dir, f"{role}.key")
    for f in (ca_file, cert_file, key_file):
        if not os.path.isfile(f):
            _log(f"ERROR: missing {f} — run streaming/generate_relay_certs.sh "
                 f"and copy the {role} cert/key + ca.crt to this machine")
            sys.exit(1)

    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ctx.load_verify_locations(cafile=ca_file)
    ctx.load_cert_chain(certfile=cert_file, keyfile=key_file)
    return ctx


# Mission Planner periodically re-sends REQUEST_DATA_STREAM (msg id 66), which
# on ArduPilot overwrites the per-message SET_MESSAGE_INTERVAL rates on the
# shared Jetson channel — tools/imu_logger.py's RAW_IMU @200 Hz collapses to
# MP's 2 Hz Sensor rate for a few seconds until the sidecar re-requests,
# tearing holes in VIO datasets. The vehicle client therefore drops msg 66 in
# the controller->vehicle direction; MP still receives telemetry at whatever
# rates launch_mavros_real.sh / the SRn params establish.
DROP_TO_VEHICLE_MSG_IDS = {66}


class MavlinkFrameFilter:
    """Stateful splitter: drops whole MAVLink frames with the given msg ids.

    Frames are delimited by the length field alone (no CRC validation, so
    unknown-dialect messages pass through intact). A byte that isn't a frame
    start (mid-frame resync garbage) is forwarded untouched.
    """

    def __init__(self, drop_ids):
        self._drop = drop_ids
        self._buf = bytearray()
        self.dropped = 0

    def feed(self, data: bytes) -> bytes:
        self._buf += data
        out = bytearray()
        while self._buf:
            b0 = self._buf[0]
            if b0 == 0xFD:                     # MAVLink 2
                if len(self._buf) < 10:
                    break
                n = 12 + self._buf[1] + (13 if self._buf[2] & 0x01 else 0)
                msg_id = self._buf[7] | self._buf[8] << 8 | self._buf[9] << 16
            elif b0 == 0xFE:                   # MAVLink 1
                if len(self._buf) < 6:
                    break
                n = 8 + self._buf[1]
                msg_id = self._buf[5]
            else:                              # not a frame start — pass through
                out.append(b0)
                del self._buf[:1]
                continue
            if len(self._buf) < n:
                break                          # incomplete frame — need more bytes
            if msg_id not in self._drop:
                out += self._buf[:n]
            else:
                self.dropped += 1
                _log(f"dropped GCS msg id {msg_id} (total {self.dropped})")
            del self._buf[:n]
        return bytes(out)


def _pump(src: socket.socket, dst: socket.socket, tag: str,
          filt: "MavlinkFrameFilter | None" = None) -> None:
    """Copy bytes src -> dst until either side closes; closes dst on exit."""
    try:
        while True:
            data = src.recv(4096)
            if not data:
                break
            if filt is not None:
                data = filt.feed(data)
                if not data:
                    continue
            dst.sendall(data)
    except OSError:
        pass
    finally:
        try:
            dst.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass


def _connect_relay(ctx: ssl.SSLContext, host: str, port: int) -> ssl.SSLSocket:
    """Blocking connect-with-backoff to the relay. Never gives up."""
    backoff = RECONNECT_BACKOFF_START
    while True:
        try:
            raw = socket.create_connection((host, port), timeout=10)
            # timeout=10 above only bounds the connect() attempt — Python leaves it set
            # on the socket afterward, which would make _pump()'s blocking recv() raise
            # socket.timeout (and tear the whole relay down) after any 10s idle gap, e.g.
            # whenever no controller is connected to send anything back. Reset to blocking.
            raw.settimeout(None)
            # Detect half-open drops (LTE/NAT dying without FIN/RST) in ~60s —
            # otherwise _pump()'s recv() blocks forever on a dead link and the
            # relay never reconnects. Mirrors the server's keepalive settings.
            raw.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            raw.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 30)
            raw.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 10)
            raw.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)
            tls = ctx.wrap_socket(raw, server_hostname=host)
            _log(f"connected to relay {host}:{port}")
            return tls
        except OSError as e:
            _log(f"relay connect failed ({e}); retrying in {backoff:.0f}s")
            time.sleep(backoff)
            backoff = min(backoff * 2, RECONNECT_BACKOFF_MAX)


def _serve_one_local_connection(local_sock: socket.socket, ctx: ssl.SSLContext,
                                 host: str, port: int, role: str) -> None:
    relay_sock = _connect_relay(ctx, host, port)
    # vehicle only: strip GCS stream-rate stomps from the inbound leg
    filt = MavlinkFrameFilter(DROP_TO_VEHICLE_MSG_IDS) if role == "vehicle" else None
    try:
        t1 = threading.Thread(target=_pump, args=(local_sock, relay_sock, "local->relay"), daemon=True)
        t2 = threading.Thread(target=_pump, args=(relay_sock, local_sock, "relay->local", filt), daemon=True)
        t1.start()
        t2.start()
        t1.join()
        t2.join()
    finally:
        for s in (local_sock, relay_sock):
            try:
                s.close()
            except OSError:
                pass


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--role", required=True, choices=["vehicle", "controller"])
    ap.add_argument("--relay-host", default="118.232.160.227")
    ap.add_argument("--relay-port", type=int, default=8683)
    ap.add_argument("--local-host", default="127.0.0.1")
    ap.add_argument("--local-port", type=int, default=5760)
    ap.add_argument("--cert-dir", default=DEFAULT_CERT_DIR)
    args = ap.parse_args()

    ctx = _build_ssl_context(args.cert_dir, args.role)

    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind((args.local_host, args.local_port))
    listener.listen(1)
    _log(f"role={args.role}  local {args.local_host}:{args.local_port}  "
         f"-> relay {args.relay_host}:{args.relay_port}")

    while True:
        local_sock, addr = listener.accept()
        _log(f"local client connected from {addr}")
        try:
            _serve_one_local_connection(local_sock, ctx, args.relay_host, args.relay_port, args.role)
        except Exception as e:
            _log(f"connection handler error: {e}")
        _log("local client disconnected — waiting for reconnect")


if __name__ == "__main__":
    main()
