#!/usr/bin/env python3
"""
MAVLink relay server — runs on Frank's PC (the same box as MediaMTX), gives
Mission Planner a second, independent path to the FC when the SiK radio is
out of range, alongside control/mavlink_relay_client.py on the Jetson
(--role vehicle) and on the Mission Planner machine (--role controller).

Requires mutual TLS: a connection is only accepted if it presents a client
cert signed by our private CA (see generate_relay_certs.sh). The cert's CN
("vehicle" or "controller") — not anything the client claims — decides
routing. Traffic from the vehicle is broadcast to every connected
controller; traffic from any controller goes only to the vehicle. There is
no application-level MAVLink parsing here — it's a byte-level hub, same
trust model as mavlink-router's TcpServer endpoint, just hand-rolled since
mavlink-router isn't apt-packaged on this box or the Jetson.

Usage:
  python3 streaming/mavlink_relay_server.py
  (needs relay_certs/{ca.crt,server.crt,server.key} — see
   generate_relay_certs.sh — copied to this machine; port 8683/tcp forwarded
   to this machine on the router, same as the existing 8554 for video)
"""

import argparse
import os
import socket
import ssl
import threading
import time
from typing import Optional, Set

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_CERT_DIR = os.path.join(PROJECT_DIR, "streaming", "relay_certs")

VALID_ROLES = ("vehicle", "controller")


def _log(msg: str) -> None:
    print(f"[relay_server:{time.strftime('%H:%M:%S')}] {msg}", flush=True)


class Hub:
    """Tracks the one vehicle connection and N controller connections."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._vehicle: Optional[socket.socket] = None
        self._controllers: Set[socket.socket] = set()

    def register(self, role: str, sock: socket.socket) -> None:
        with self._lock:
            if role == "vehicle":
                old = self._vehicle
                self._vehicle = sock
                if old is not None:
                    _log("new vehicle connection replacing previous one")
                    try:
                        old.close()
                    except OSError:
                        pass
            else:
                self._controllers.add(sock)

    def unregister(self, role: str, sock: socket.socket) -> None:
        with self._lock:
            if role == "vehicle" and self._vehicle is sock:
                self._vehicle = None
            else:
                self._controllers.discard(sock)

    def from_vehicle(self, data: bytes) -> None:
        with self._lock:
            targets = list(self._controllers)
        for c in targets:
            try:
                c.sendall(data)
            except OSError:
                pass  # that controller's own recv loop will notice and clean up

    def from_controller(self, data: bytes) -> None:
        with self._lock:
            v = self._vehicle
        if v is not None:
            try:
                v.sendall(data)
            except OSError:
                pass


def _peer_role(tls_sock: ssl.SSLSocket) -> Optional[str]:
    cert = tls_sock.getpeercert()
    if not cert:
        return None
    for rdn in cert.get("subject", ()):
        for key, value in rdn:
            if key == "commonName" and value in VALID_ROLES:
                return value
    return None


def _handle_connection(tls_sock: ssl.SSLSocket, addr, hub: Hub) -> None:
    role = _peer_role(tls_sock)
    if role is None:
        _log(f"rejecting {addr}: client cert CN is not a recognized role")
        tls_sock.close()
        return

    _log(f"{role} connected from {addr}")
    hub.register(role, tls_sock)
    route = hub.from_vehicle if role == "vehicle" else hub.from_controller
    try:
        while True:
            data = tls_sock.recv(4096)
            if not data:
                break
            route(data)
    except OSError:
        pass
    finally:
        hub.unregister(role, tls_sock)
        try:
            tls_sock.close()
        except OSError:
            pass
        _log(f"{role} disconnected ({addr})")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bind-host", default="0.0.0.0")
    ap.add_argument("--bind-port", type=int, default=8683)
    ap.add_argument("--cert-dir", default=DEFAULT_CERT_DIR)
    args = ap.parse_args()

    ca_file   = os.path.join(args.cert_dir, "ca.crt")
    cert_file = os.path.join(args.cert_dir, "server.crt")
    key_file  = os.path.join(args.cert_dir, "server.key")
    for f in (ca_file, cert_file, key_file):
        if not os.path.isfile(f):
            _log(f"ERROR: missing {f} — run generate_relay_certs.sh and copy "
                 f"ca.crt/server.crt/server.key to this machine")
            raise SystemExit(1)

    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(certfile=cert_file, keyfile=key_file)
    ctx.load_verify_locations(cafile=ca_file)
    ctx.verify_mode = ssl.CERT_REQUIRED  # reject any connection without a valid client cert

    hub = Hub()

    raw_listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    raw_listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    raw_listener.bind((args.bind_host, args.bind_port))
    raw_listener.listen(8)
    _log(f"listening on {args.bind_host}:{args.bind_port} (mTLS, client cert required)")

    while True:
        raw_sock, addr = raw_listener.accept()
        try:
            tls_sock = ctx.wrap_socket(raw_sock, server_side=True)
        except ssl.SSLError as e:
            _log(f"TLS handshake failed from {addr}: {e}")
            raw_sock.close()
            continue
        threading.Thread(target=_handle_connection, args=(tls_sock, addr, hub),
                          daemon=True).start()


if __name__ == "__main__":
    main()
