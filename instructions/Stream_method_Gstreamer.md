# Gstreamer_opencv.py — Debug & Fix Session

## Overview

Script: `Gstreamer_opencv.py`  
Platform: NVIDIA Jetson (JetPack 36.x, aarch64)  
Goal: USB camera → OpenCV face detection → H.265 hardware encode → RTP/UDP stream to ground station

---

## Issues Found & Fixed

### 1. Missing `nvidia-l4t-gstreamer` package

**Symptom:** `WARNING: erroneous pipeline: no element "nvvidconv"`  
**Cause:** `nvidia-l4t-gstreamer` was not installed. This package provides the Jetson-specific GStreamer elements (`nvvidconv`, `nvv4l2h265enc`, etc.). Only `nvidia-l4t-multimedia` (the base library) was present.  
**Fix:**
```bash
sudo apt install nvidia-l4t-gstreamer
```

---

### 2. Haar Cascade file not found

**Symptom:** `Error: Cascade file not found at haarcascade_frontalface_default.xml`  
**Cause:** Script used a bare filename with no path. The file is bundled with OpenCV at `cv2.data.haarcascades`.  
**Fix:**
```python
# Before
cascade_path = "haarcascade_frontalface_default.xml"

# After
cascade_path = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
```

---

### 3. Invalid `preset-level` value

**Symptom:** `WARNING: could not set property "preset-level" in element "nvv4l2h265enc0" to "Ultrafast"`  
**Cause:** Wrong enum string. The valid value is `UltraFastPreset` not `Ultrafast`.  
**Fix:**
```
# Before
preset-level=Ultrafast

# After
preset-level=UltraFastPreset
```

---

### 4. `NvBufSurfaceCopy: failed` — fdsrc incompatible with NVMM (root cause)

**Symptom:** `nvbufsurface: NvBufSurfaceCopy: mem copy failed` on every frame. No video encoded.  
**Cause:** The original script used `subprocess + gst-launch-1.0` with `fdsrc` reading raw bytes from Python's `pipe.stdin`. `fdsrc` allocates standard heap memory buffers. On JetPack 36.x, `nvvidconv` cannot DMA-copy heap-allocated buffers into NVMM (GPU memory) — required by `nvv4l2h265enc`.

This was confirmed by testing:
- `fdsrc → nvvidconv → NVMM` → fails with `NvBufSurfaceCopy` error on every frame
- `v4l2src → nvvidconv → NVMM` → works perfectly (V4L2 allocates DMA-compatible buffers)
- `appsrc → videoconvert → nvvidconv → NVMM` → works (videoconvert re-allocates into a GStreamer buffer pool compatible with nvvidconv)

**Fix:** Rewrote the script to use `gi.repository.Gst` (GStreamer Python bindings) with `appsrc` instead of `subprocess + fdsrc`. The `videoconvert` element between `appsrc` and `nvvidconv` bridges the memory gap.

```python
# Before (broken)
import subprocess
gst_cmd = ['gst-launch-1.0', 'fdsrc', '!', ...]
pipe = subprocess.Popen(gst_cmd, stdin=subprocess.PIPE)
pipe.stdin.write(frame.tobytes())

# After (working)
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

pipeline = Gst.parse_launch(
    'appsrc name=src ... ! videoconvert ! nvvidconv ! '
    'video/x-raw(memory:NVMM),format=NV12 ! nvv4l2h265enc ...'
)
appsrc = pipeline.get_by_name('src')
buf = Gst.Buffer.new_wrapped(frame.tobytes())
appsrc.emit('push-buffer', buf)
```

---

### 5. Black view on receiver — resolution mismatch

**Symptom:** Receiver showed a black screen after lowering resolution to 480p.  
**Cause:** Resolution set to 854×480 in the pipeline, but the USB camera's closest supported mode is **848×480**. Buffer size mismatch → corrupted/black frames on receiver.  
**Fix:** Set `width = 848` to match the camera's actual output.

```python
# Before
width = 854
height = 480

# After
width = 848
height = 480
```

Always verify the camera's actual resolution after setting:
```python
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
ret, frame = cap.read()
print(frame.shape)  # must match width/height
```

---

### 6. Gray view with noise on modem — H.265 sync loss from packet loss

**Symptom:** Receiver shows gray noise when Jetson switches from WiFi to modem.  
**Cause:** Default `idrinterval=256` means an IDR keyframe only every ~8.5 seconds at 30fps. On a lossy modem path, losing one IDR frame causes the decoder to show gray for up to 8.5 seconds. Additionally, large RTP packets may be fragmented across different MTU paths (modem vs WiFi relay), and any lost fragment drops the whole packet.  

**Fix:**
```python
# Increased IDR frequency: keyframe every 1 second (30 frames)
# Reduced bitrate to fit modem uplink
# Reduced MTU to prevent IP fragmentation across relay paths
'nvv4l2h265enc bitrate=1000000 preset-level=UltraFastPreset idrinterval=30 iframeinterval=30 ! '
'rtph265pay config-interval=-1 mtu=1200 ! '
```

| Parameter | Before | After | Reason |
|---|---|---|---|
| `bitrate` | 2000000 | 1000000 | Fits modem uplink bandwidth |
| `idrinterval` | 256 (default) | 30 | Decoder recovers within 1s of packet loss |
| `iframeinterval` | 30 (default) | 30 | Explicit, aligned with IDR |
| `config-interval` | 1 | -1 | SPS/PPS sent with every IDR for immediate resync |
| `mtu` | 1400 (default) | 1200 | Prevents IP fragmentation on relay paths |

---

## ZeroTier Direct Path (Port 9993)

When Jetson is on modem and receiver is on WiFi, ZeroTier may route packets through a relay server (high latency/packet loss). Opening UDP port 9993 allows ZeroTier to establish a **direct peer-to-peer path**.

### Jetson firewall
```bash
sudo iptables -A INPUT -p udp --dport 9993 -j ACCEPT
sudo iptables -A OUTPUT -p udp --sport 9993 -j ACCEPT

# Persist across reboots
sudo apt install iptables-persistent -y
sudo netfilter-persistent save
```

### Router port forwarding
In your router's admin page (`192.168.1.1`), add a port forwarding rule:

| Field | Value |
|---|---|
| Protocol | UDP |
| External port | 9993 |
| Internal port | 9993 |
| Internal IP | Jetson's local IP |

Do the same on the receiver's router, pointing to the receiver's local IP.

### Verify direct path
```bash
sudo zerotier-cli peers
# Look for DIRECT (good) vs RELAY (packet loss likely)
```

### Find local IP
```bash
ip route get 1 | awk '{print $7; exit}'
```

---

## Final Script

```python
import cv2
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
import sys

Gst.init(None)

# Pipeline Configuration
ground_ip = "10.181.156.237"
width = 848
height = 480
fps = 30

# Load Haar Cascade for face detection
cascade_path = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
face_cascade = cv2.CascadeClassifier(cascade_path)

# Encode pipeline: appsrc → videoconvert → nvvidconv(NVMM) → nvv4l2h265enc → RTP → UDP
encode_pipeline = Gst.parse_launch(
    f'appsrc name=src format=time is-live=true block=true '
    f'caps=video/x-raw,format=BGR,width={width},height={height},framerate={fps}/1 ! '
    f'videoconvert ! '
    f'nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! '
    f'nvv4l2h265enc bitrate=1000000 preset-level=UltraFastPreset idrinterval=30 iframeinterval=30 ! '
    f'rtph265pay config-interval=-1 mtu=1200 ! '
    f'udpsink host={ground_ip} port=5000 sync=false'
)
appsrc = encode_pipeline.get_by_name('src')
encode_pipeline.set_state(Gst.State.PLAYING)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
cap.set(cv2.CAP_PROP_FPS, fps)

if not cap.isOpened():
    print("Error: Could not open camera.")
    encode_pipeline.set_state(Gst.State.NULL)
    sys.exit(1)

print("Face detection streaming started. Press Ctrl+C to stop.")

frame_count = 0
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Camera read failed.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(
            gray, scaleFactor=1.1, minNeighbors=3, minSize=(30, 30)
        )
        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(
            frame, f"Faces Detected: {len(faces)}",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2
        )

        buf = Gst.Buffer.new_wrapped(frame.tobytes())
        buf.pts = frame_count * Gst.SECOND // fps
        buf.duration = Gst.SECOND // fps
        flow = appsrc.emit('push-buffer', buf)
        if flow != Gst.FlowReturn.OK:
            print(f"Encoder pipeline error: {flow}")
            break
        frame_count += 1

except KeyboardInterrupt:
    print("\nStopping stream...")

finally:
    cap.release()
    appsrc.emit('end-of-stream')
    encode_pipeline.set_state(Gst.State.NULL)
    print("Stream closed successfully.")
```

---

## Receiver GStreamer Pipeline (ground station)

```bash
gst-launch-1.0 udpsrc port=5000 ! \
  application/x-rtp,encoding-name=H265,payload=96 ! \
  rtph265depay ! h265parse ! avdec_h265 ! \
  videoconvert ! autovideosink sync=false
```

---

## Key Takeaways

- `nvidia-l4t-gstreamer` must be installed separately from `nvidia-l4t-multimedia` to get Jetson hardware GStreamer elements.
- `fdsrc` (subprocess pipe) allocates heap memory incompatible with NVMM DMA on JetPack 36.x. Use `gi.repository.Gst` with `appsrc + videoconvert` instead.
- Always verify camera resolution with `cap.read()` after `cap.set()` — cameras round to their nearest supported mode.
- For streaming over lossy links, set `idrinterval` low (30 = 1s recovery), `config-interval=-1`, and `mtu=1200` to prevent fragmentation.
- ZeroTier `RELAY` mode causes packet loss. Open UDP 9993 on both routers for a direct path.
