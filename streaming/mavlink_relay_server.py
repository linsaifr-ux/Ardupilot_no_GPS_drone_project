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
import queue
import socket
import ssl
import threading
import time
from typing import Optional, Set

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_CERT_DIR = os.path.join(PROJECT_DIR, "streaming", "relay_certs")

VALID_ROLES = ("vehicle", "controller")

# Outbound chunks a peer may fall behind before we declare it dead (~2 MB at
# the 4 KB recv size — MAVLink telemetry is ~10 KB/s, so a healthy peer never
# gets near this).
SEND_QUEUE_MAX = 512


def _log(msg: str) -> None:
    print(f"[relay_server:{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def _enable_keepalive(raw_sock: socket.socket) -> None:
    """Detect half-open TCP connections (peer vanished without FIN/RST — LTE
    drop, NAT timeout, machine sleep) in ~60 s instead of never. Without this
    a dead peer's socket looks healthy until its send buffer fills."""
    raw_sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    raw_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 30)
    raw_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 10)
    raw_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)


class Connection:
    """One authenticated peer: owns the socket plus a writer thread draining a
    bounded queue.

    This exists because the original Hub called blocking sendall() on every
    controller from the *vehicle's* recv thread — one controller going
    half-open (never errors, just stops ACKing) filled its send buffer and
    wedged the entire relay for everyone (observed 2026-07-09: MP could
    authenticate but received nothing; vehicle client backed up 222 KB).
    With a writer thread per peer, a stalled peer only ever blocks its own
    writer; when its queue fills, the peer is evicted.
    """

    def __init__(self, sock: ssl.SSLSocket, role: str, addr) -> None:
        self.sock = sock
        self.role = role
        self.addr = addr
        self._q: queue.Queue = queue.Queue(SEND_QUEUE_MAX)
        threading.Thread(target=self._drain, daemon=True,
                         name=f"writer-{role}-{addr}").start()

    def send(self, data: bytes) -> bool:
        """Never blocks. Returns False if the peer's queue is full — the
        caller (Hub) must then evict it: dead or hopelessly slow."""
        try:
            self._q.put_nowait(data)
            return True
        except queue.Full:
            return False

    def _drain(self) -> None:
        while True:
            data = self._q.get()
            if data is None:        # sentinel from stop()
                break
            try:
                self.sock.sendall(data)
            except OSError:
                break
        self.close()

    def stop(self) -> None:
        """Ask the writer to exit once the queue drains, then close."""
        try:
            self._q.put_nowait(None)
        except queue.Full:
            pass                    # writer is stuck in sendall — close() below unblocks it
        self.close()

    def close(self) -> None:
        # shutdown() first: on Linux, close() alone does NOT wake a thread
        # blocked in recv() on this socket — the handler thread would sit
        # there forever and never unregister the connection.
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            self.sock.close()
        except OSError:
            pass


class Hub:
    """Tracks the one vehicle connection and N controller connections."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._vehicle: Optional[Connection] = None
        self._controllers: Set[Connection] = set()

    def register(self, conn: Connection) -> None:
        with self._lock:
            if conn.role == "vehicle":
                old = self._vehicle
                self._vehicle = conn
                if old is not None:
                    _log("new vehicle connection replacing previous one")
                    old.stop()
            else:
                self._controllers.add(conn)

    def unregister(self, conn: Connection) -> None:
        with self._lock:
            if conn.role == "vehicle" and self._vehicle is conn:
                self._vehicle = None
            else:
                self._controllers.discard(conn)

    def _evict(self, conn: Connection) -> None:
        """Remove a backed-up peer immediately — don't wait for its recv loop
        (a half-open peer's recv never returns on its own)."""
        _log(f"{conn.role} {conn.addr}: send queue full — evicting dead/slow peer")
        self.unregister(conn)
        conn.stop()

    def from_vehicle(self, data: bytes) -> None:
        with self._lock:
            targets = list(self._controllers)
        for c in targets:
            if not c.send(data):    # non-blocking
                self._evict(c)

    def from_controller(self, data: bytes) -> None:
        with self._lock:
            v = self._vehicle
        if v is not None and not v.send(data):
            self._evict(v)


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
    conn = Connection(tls_sock, role, addr)
    hub.register(conn)
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
        hub.unregister(conn)
        conn.stop()
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
        _enable_keepalive(raw_sock)
        try:
            tls_sock = ctx.wrap_socket(raw_sock, server_side=True)
        except (ssl.SSLError, OSError) as e:
            _log(f"TLS handshake failed from {addr}: {e}")
            raw_sock.close()
            continue
        threading.Thread(target=_handle_connection, args=(tls_sock, addr, hub),
                          daemon=True).start()


if __name__ == "__main__":
    main()
