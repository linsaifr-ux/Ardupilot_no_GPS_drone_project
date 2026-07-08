# MAVLink relay — redundant Mission Planner link over the internet

Gives Mission Planner a second, independent path to the FC (telemetry **and**
control — arm/disarm/RTL/mode/param) alongside the existing 915MHz SiK radio,
routed through the Jetson's own FC link (`SERIAL6`) and Frank's PC
(`118.232.160.227`, the same box that runs MediaMTX for the video stream).

```
FC ──SERIAL6── Jetson (mavros, gcs_url) ──mTLS──▶ Frank's PC (relay hub) ◀──mTLS── Mission Planner machine
```

Authenticated with mutual TLS (mTLS) — a connection is only accepted if it
presents a client certificate signed by a small private CA generated once
for this project. There's no FC-level MAVLink signing involved (it's
incompatible with mavros — see `instructions/how_to_run_real_hw.md` for why)
— the FC and mavros are completely unaware this relay exists; it just looks
like a normal second GCS bridge from mavros's side.

**Do not use this as your only link on flight day.** It depends on the
Jetson's LTE connection and Frank's PC being reachable — the SiK radio
remains the primary, always-on link. Treat this purely as a backup for when
the radio drops out of range.

**Known limitation — no `STATUSTEXT` (prearm/warning messages).** Mission
Planner's Messages tab will stay empty on this connection even though
telemetry and control both work fully. This is permanent, not a bug:
`SERIAL6_OPTIONS=1024` (set to stop the commander's VPE stream from
flooding the radio link, see `instructions/how_to_run_real_hw.md`) marks
the Jetson's FC link "private," and ArduPilot's `STATUSTEXT` distribution
unconditionally excludes private channels — confirmed via ArduPilot source
and a live prearm-failure test. Don't try to fix this by clearing
`SERIAL6_OPTIONS` — it reopens the VPE-flooding problem. If you need
prearm-type health info on this link, that would mean building a small
status line from mavros's structured `/mavros/sys_status` data instead —
ask if you want that.

## 1. Generate certificates (once, any machine with `openssl`)

```bash
bash streaming/generate_relay_certs.sh 118.232.160.227
```

Produces `streaming/relay_certs/{ca,server,vehicle,controller}.{crt,key}`.
These are secrets — gitignored, never commit them. Copy by hand:

| File(s) | Goes to |
|---|---|
| `ca.crt`, `server.crt`, `server.key` | Frank's PC (relay server) |
| `ca.crt`, `vehicle.crt`, `vehicle.key` | Jetson (`streaming/relay_certs/` in this repo) |
| `ca.crt`, `controller.crt`, `controller.key` | Mission Planner machine (`streaming/relay_certs/` if the repo is checked out there, otherwise anywhere — path is a `--cert-dir` flag) |

## 2. Frank's PC — run the relay server

```bash
python3 streaming/mavlink_relay_server.py
```

Listens on `0.0.0.0:8683`, mTLS, rejects anything without a valid client
cert. Port-forward **8683/tcp** on the router to this machine's LAN IP
(`192.168.0.134`), same as the existing 8554 forward for RTSP.

## 3. Jetson — run the relay client (role=vehicle)

Either standalone:
```bash
python3 control/mavlink_relay_client.py --role vehicle
```
or automatically as part of the full stack:
```bash
bash control/launch_real_hw.sh --mavlink-relay
```
which also sets `MAVLINK_RELAY=1` for `launch_mavros_real.sh`, adding
`gcs_url:="tcp://127.0.0.1:5760"` to mavros so it bridges its full FCU
stream to the relay client.

## 4. Mission Planner machine — run the relay client (role=controller)

```bash
python3 control/mavlink_relay_client.py --role controller
```
(pure-Python, stdlib only — no extra packaging needed on Windows/Linux/Mac,
just Python 3 installed)

Then in Mission Planner, add a new **TCP** connection: `127.0.0.1:5760`.
Connect to it alongside (not instead of) your existing radio COM-port
connection — MP supports multiple simultaneous links.

## Testing before you trust it

1. Confirm telemetry: with the relay's TCP connection selected in MP, you
   should see live position/battery/mode, same as the radio link.
2. **Props off, vehicle restrained:** test a mode change or param refresh
   over the relay link first, confirm it actually took effect (check the
   Jetson's mavros logs or `/mavros/state`), not just that MP's UI *looked*
   like it sent something.
3. Only after that, test arm/disarm the same way (props off) before ever
   relying on RTL-via-relay in flight.
4. Pull the SiK radio's antenna to confirm MP keeps working on the relay
   connection alone — this is the actual redundancy this whole thing is for.

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `mavlink_relay_client.py` / `mavlink_relay_server.py`: `ERROR: missing .../ca.crt` | Certs not generated/copied to this machine yet | Run step 1, copy the right files per the table above |
| Relay client stuck retrying `relay connect failed` | Port 8683 not reachable from this machine | Check Frank's PC is running the server and the router forwards 8683/tcp; `nc -zv 118.232.160.227 8683` |
| MP shows the TCP connection but no telemetry | `mavlink_relay_client.py --role controller` not running locally, or mavros's `gcs_url` not enabled | Confirm both processes are running; confirm `MAVLINK_RELAY=1` was set when `launch_mavros_real.sh` started (restart it if not) |
| `relay_server.py` log: `rejecting <addr>: client cert CN is not a recognized role` | Wrong cert copied (e.g. `server.crt` used as a client cert), or cert generated by a different/old CA run | Re-copy the correct `vehicle.crt`/`controller.crt`, or regenerate all certs and redistribute to all three machines together (mixing certs from different CA runs will never validate) |
| Relay connects fine but flaps every ~10s (`mavconn: tcp3: send: channel closed!` in mavros) | Fixed 2026-07-08 — `_connect_relay()` left a 10s socket timeout active after connecting, so idle periods (e.g. no controller connected yet) spuriously tore the link down | Pull latest `control/mavlink_relay_client.py`; if it recurs, check `_connect_relay()` calls `raw.settimeout(None)` after `create_connection()` |
| Mission Planner's Messages tab is empty on the relay connection | Expected — see "Known limitation" above, not a fault | No fix needed; check the radio connection's Messages tab instead, or use `/mavros/sys_status` for structured health data |
