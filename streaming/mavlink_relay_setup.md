# MAVLink relay — redundant Mission Planner link over the internet

Gives Mission Planner a second, independent path to the FC (telemetry **and**
control — arm/disarm/RTL/mode/param) alongside the existing 915MHz SiK radio,
routed through the Jetson's own FC link and Frank's PC
(`118.232.160.227`, the same box that runs MediaMTX for the video stream).
The FC-side port of that link depends on the flight controller: `SERIAL1`
(TELEM1) on the current Pixhawk 6C (since the 2026-07-21 FC swap), `SERIAL6`
on the previous FC.

```
FC ──SERIAL1 (TELEM1)── Jetson (mavros, gcs_url) ──mTLS──▶ Frank's PC (relay hub) ◀──mTLS── Mission Planner machine
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
the Jetson port's `SERIALn_OPTIONS=1024` (`SERIAL1_OPTIONS` on the current
Pixhawk 6C, set by `control/real_hw_pixhawk6c.parm` to stop the commander's
VPE stream from flooding the radio link, see
`instructions/how_to_run_real_hw.md`) marks
the Jetson's FC link "private," and ArduPilot's `STATUSTEXT` distribution
unconditionally excludes private channels — confirmed via ArduPilot source
and a live prearm-failure test. Don't try to fix this by clearing
that option — it reopens the VPE-flooding problem. If you need
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

This machine runs a *copy* of the file — updates to it in the repo do
nothing until re-copied here and the server restarted. Must be the
2026-07-09+ version: earlier versions wedge permanently for all peers the
first time any controller drops without a TCP close (see troubleshooting).
The current version evicts backed-up peers (logged as `send queue full —
evicting dead/slow peer`) and uses TCP keepalive to reap half-open sockets.

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
stream to the relay client. The Desktop `field_data_collection.sh` launcher
does the same (relay client + `MAVLINK_RELAY=1`, added 2026-07-21), so MP
can watch telemetry during data-collection flights too; it also pkills any
stale vehicle client from a previous session before starting its own.

**The vehicle client filters both directions** (added 2026-07-21):

- *Inbound (GCS→vehicle): drops `REQUEST_DATA_STREAM` (msg id 66).* Mission
  Planner re-sends its stream-rate requests in bursts every ~15 s; on the
  shared Jetson channel those overwrote the `SET_MESSAGE_INTERVAL` rates
  `tools/imu_logger.py` needs, collapsing the 200 Hz RAW_IMU recording
  stream to MP's 2 Hz Sensor rate for a few seconds at a time (holes in VIO
  datasets). Side effect: MP cannot *change* telemetry stream rates over the
  relay (they come from `launch_mavros_real.sh`'s 10 Hz request and the
  `SRn_*` params). Verified live against real MP: RAW_IMU held 200 Hz
  through the bursts.
- *Outbound (vehicle→GCS): drops `RAW_IMU` (27) and `ATTITUDE_QUATERNION`
  (31).* mavros mirrors the whole FCU stream to the gcs bridge, so during
  recording the 200 Hz + 50 Hz sidecar streams (~29 KB/s of TCP) went over
  the same LTE uplink as the MediaMTX video push and saturated it — measured
  2026-07-21: video send backlog ~650 KB (≈5–6 s of stream lag), RTT 32 ms →
  1.4 s, ~6 % retransmissions. With the filter the relay idles at ~11 KB/s
  (the normal 10 Hz telemetry set) and RTT returned to ~50 ms. MP loses
  nothing it displays — its HUD uses the 10 Hz `ATTITUDE` message.

The log lines `dropped GCS/outbound msg id N (total M)` are these filters
working — normal whenever MP is connected, not an error (high-rate outbound
drops are logged only every 5000th). Everything else — arm, mode, RTL,
params, mission upload — passes through unmodified; command round-trips
verified unaffected.

**If you run the client standalone, mavros must also be started with
`MAVLINK_RELAY=1 bash control/launch_mavros_real.sh`** — the relay client by
itself carries no data; it just bridges whatever mavros feeds it. Forgetting
the env var is a silent per-boot failure: every relay piece looks healthy
(client running, server reachable, MP's TCP connect succeeds) but no
heartbeat ever flows, so MP times out. This exact miss happened 2026-07-09.
Quick check on the Jetson: `pgrep -af mavros_node` — the args must include
`gcs_url`, otherwise restart mavros with the env var.

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
| MP shows the TCP connection but no telemetry / MP connect times out while every relay piece looks up | `mavlink_relay_client.py --role controller` not running locally, or mavros's `gcs_url` not enabled — most likely `launch_mavros_real.sh` was started without `MAVLINK_RELAY=1` (bit us 2026-07-09) | Confirm both processes are running; on the Jetson check `pgrep -af mavros_node` shows a `gcs_url` arg — if not, restart mavros as `MAVLINK_RELAY=1 bash control/launch_mavros_real.sh` (the vehicle relay client can stay running) |
| `relay_server.py` log: `rejecting <addr>: client cert CN is not a recognized role` | Wrong cert copied (e.g. `server.crt` used as a client cert), or cert generated by a different/old CA run | Re-copy the correct `vehicle.crt`/`controller.crt`, or regenerate all certs and redistribute to all three machines together (mixing certs from different CA runs will never validate) |
| Relay connects fine but flaps every ~10s (`mavconn: tcp3: send: channel closed!` in mavros) | Fixed 2026-07-08 — `_connect_relay()` left a 10s socket timeout active after connecting, so idle periods (e.g. no controller connected yet) spuriously tore the link down | Pull latest `control/mavlink_relay_client.py`; if it recurs, check `_connect_relay()` calls `raw.settimeout(None)` after `create_connection()` |
| Everything *connects* (vehicle client up, MP's TLS accepted) but MP gets no data and times out; Jetson-side `ss` shows the vehicle client's Send-Q/Recv-Q piling up | Fixed 2026-07-09 — one earlier controller connection died without a TCP close (LTE/NAT drop, killed process); the server's blocking `sendall()` to that half-open socket wedged the whole hub for everyone | Restart the relay server on Frank's PC (instant un-wedge) **and update it to the latest `streaming/mavlink_relay_server.py`** — per-peer writer threads + bounded queues evict dead peers instead of wedging, TCP keepalive detects half-open drops in ~60s. Also restart the Jetson vehicle client once to pick up its matching keepalive fix |
| Mission Planner's Messages tab is empty on the relay connection | Expected — see "Known limitation" above, not a fault | No fix needed; check the radio connection's Messages tab instead, or use `/mavros/sys_status` for structured health data |
| Vehicle client exits immediately with `OSError: [Errno 98] Address already in use` — or a client-code fix seems to have no effect | An older relay client still owns port 5760. Classic case (bit us 2026-07-21): `field_data_collection.sh` sitting at its final "Press Enter to close" prompt — its EXIT trap hasn't fired yet, so its (possibly old-code) client keeps serving while the new one silently isn't in the path | `ss -tlnp \| grep 5760` to see which PID owns the port; close the old launcher window (or kill that PID), then start the new client |
| Recorded IMU rate dips to ~2 Hz while MP is connected via relay | `REQUEST_DATA_STREAM` stomps reaching the FC — the vehicle client in the path predates the 2026-07-21 msg-66 filter (see above; often the stale-client row is the real cause) | Make sure the *current* `control/mavlink_relay_client.py` is the one bound to 5760; its log must show `dropped GCS msg id 66` lines while MP is connected |
| Ground-view video stream turns laggy (seconds of delay) whenever the relay runs alongside recording | 200 Hz RAW_IMU forwarded to MP saturating the shared LTE uplink — vehicle client predates the 2026-07-21 outbound filter (or is a stale instance, see above) | Current client drops msgs 27/31 outbound (log shows `dropped outbound msg id 27`); check `ss -tin dst 118.232.160.227` — relay leg should sit around ~11 KB/s, RTT tens of ms, video leg Send-Q near zero |
