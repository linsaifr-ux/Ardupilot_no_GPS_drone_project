#!/bin/bash
# One-time setup: generates a private CA + certs for the mTLS-authenticated
# MAVLink relay (mavlink_relay_server.py on Frank's PC, mavlink_relay_client.py
# on the Jetson and on the Mission Planner machine).
#
# Run this ONCE, anywhere with openssl (doesn't need to be on the relay
# server itself). Produces relay_certs/{ca,server,vehicle,controller}.{crt,key}
# plus relay_certs/ca.crt (needed by all three machines to verify each other).
#
# After running, copy files by hand — none of this is committed to git
# (relay_certs/ is gitignored, these are secrets):
#   Frank's PC (relay server) : ca.crt, server.crt, server.key
#   Jetson (vehicle leg)      : ca.crt, vehicle.crt, vehicle.key
#   MP machine (controller)   : ca.crt, controller.crt, controller.key
#
# Usage:
#   bash streaming/generate_relay_certs.sh [SERVER_IP]
#   SERVER_IP defaults to 118.232.160.227 (the existing relay), used as the
#   server cert's Subject Alternative Name so clients can verify the hostname.

set -e
SERVER_IP="${1:-118.232.160.227}"
OUT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/relay_certs"
DAYS=3650   # 10 years — this is a closed, single-purpose private CA, not a
            # public one; long validity avoids re-provisioning three machines
            # mid-project. Regenerate + redistribute if a key is ever suspected leaked.

mkdir -p "$OUT_DIR"
cd "$OUT_DIR"

echo "[certs] Output directory: $OUT_DIR"

# 1. Private CA — signs the server cert and both client certs. Nothing
#    outside these three machines needs to trust it.
openssl genrsa -out ca.key 4096 2>/dev/null
openssl req -x509 -new -nodes -key ca.key -sha256 -days "$DAYS" \
    -subj "/CN=drone-mavlink-relay-ca" -out ca.crt

# 2. Server cert (Frank's PC) — CN + SAN carry the relay's IP so clients can
#    verify the hostname during the TLS handshake.
openssl genrsa -out server.key 2048 2>/dev/null
openssl req -new -key server.key -subj "/CN=$SERVER_IP" -out server.csr
openssl x509 -req -in server.csr -CA ca.crt -CAkey ca.key -CAcreateserial \
    -days "$DAYS" -sha256 \
    -extfile <(printf "subjectAltName=IP:%s" "$SERVER_IP") \
    -out server.crt
rm -f server.csr

# 3. Client certs — CN is the role string. mavlink_relay_server.py reads the
#    CN off the verified cert to decide whether a connection is the vehicle
#    or a controller; it does not trust anything the client says about itself
#    outside the cert.
for role in vehicle controller; do
    openssl genrsa -out "$role.key" 2048 2>/dev/null
    openssl req -new -key "$role.key" -subj "/CN=$role" -out "$role.csr"
    openssl x509 -req -in "$role.csr" -CA ca.crt -CAkey ca.key -CAcreateserial \
        -days "$DAYS" -sha256 -out "$role.crt"
    rm -f "$role.csr"
done

rm -f ca.srl
chmod 600 ./*.key

echo "[certs] Done. Generated:"
ls -1 "$OUT_DIR"
echo
echo "[certs] Copy to each machine (see header comment for exact file lists),"
echo "        then delete $OUT_DIR here unless this machine is one of the three."
