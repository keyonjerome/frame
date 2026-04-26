# gst_recording_daemon

`gst_recording_daemon` is a standalone C++17 Linux daemon for recording an HDMI-to-USB capture device with GStreamer while triggering Z-CAM local recording over Ethernet. It is intentionally minimal and does not use ROS 2.

## Dependencies

Install the build dependencies:

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential \
  cmake \
  pkg-config \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev
```

Install useful runtime packages:

```bash
sudo apt-get install -y \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav \
  v4l-utils \
  socat
```

The runtime GStreamer install must provide `v4l2src`, `jpegdec`, `videoconvert`, `x264enc`, `h264parse`, `mp4mux`, `srtsrc`, `tsdemux`, `aacparse`, and `matroskamux`. `x264enc` normally comes from `gstreamer1.0-plugins-ugly`. On some Ubuntu builds, SRT support may require additional GStreamer bad plugin packages or SRT-enabled plugin builds.

## Build

```bash
cd frame/gst_recording_daemon
mkdir -p build
cd build
cmake ..
cmake --build .
```

## Run

```bash
cd frame/gst_recording_daemon/build
./gst_recording_daemon --device /dev/video0 --camera-host 10.98.32.1 --fps 60
```

Optional flags:

```bash
./gst_recording_daemon --device /dev/video2 --camera-host 10.98.32.1 --fps 30 --socket-path /tmp/filmer_recorder.sock
```

`--device` is the HDMI USB capture device. The default is `/dev/video0`.

For the tested HDMI capture card, `gst-device-monitor-1.0 Video/Source` reports it as `USB Video (V4L2)` at `/dev/video2` with 1920x1080 MJPEG support. Use `/dev/video2`, not the integrated laptop camera at `/dev/video0`.

`--camera-host` is the Z-CAM camera IP or host. The default is `10.98.32.1`.

The daemon records HDMI first. After host capture starts, it sends `/ctrl/rec?action=start` so the Z-CAM records locally to its attached media. If HDMI capture fails, the daemon falls back to the camera SRT endpoint and records the MPEG-TS payload to MKV. SRT fallback uses the camera's `stream1` profile and does not switch the camera to `Stream0`, because `Stream0` can conflict with camera-local recording.

For HDMI capture, the daemon probes the V4L2 device and selects the supported mode closest to 3840x2160 at 30 fps. The current test capture card only exposes 1920x1080 MJPEG as its best 16:9 mode, but a future 4K30-capable card should be selected automatically.

## Protocol

The daemon listens on a Unix domain socket:

```text
/tmp/filmer_recorder.sock
```

Commands:

- `PING`
- `STATUS`
- `START <output_dir>`
- `STOP`

Notes:

- `START` expects a directory, not a filename.
- The daemon creates a timestamped file like `recording_20250101_120000.mp4` for HDMI capture, or `.mkv` when SRT fallback is used.
- `STATUS` returns JSON and synchronously includes fresh camera `/info` health.
- `HEARTBEAT` returns JSON without camera information.
- Only one client is supported at a time. Extra clients are rejected while one is connected.
- Clean stop via EOS is used so the muxer can flush final metadata.

Example `STATUS` response:

```json
{"type":"status","state":"RECORDING","healthy":true,"current_file":"/tmp/recordings/recording_20260425_120000.mp4","last_error":"","capture_path":"hdmi","local_recording_warning":"","camera":{"ok":true,"info":{},"mode":"rec_ing","media":{"remain_minutes":"85","free":"159267","total":"953830","dcim_visible":true}}}
```

Example `HEARTBEAT` response:

```json
{"type":"heartbeat","state":"RECORDING","healthy":true,"current_file":"/tmp/recordings/recording_20260425_120000.mp4","capture_path":"hdmi"}
```

## Quick Tests

### Find the Capture Device

List V4L2 devices and identify the HDMI capture card:

```bash
ls -l /dev/video*
gst-device-monitor-1.0 Video/Source
```

On the current test machine, the integrated laptop camera is `/dev/video0` and the HDMI capture card is:

```text
USB Video (V4L2)
api.v4l2.path = /dev/video2
```

Optional direct V4L2 check:

```bash
v4l2-ctl --device /dev/video2 --list-formats-ext
```

### Start the Daemon

From the repository root:

```bash
cd /home/keyon/code/frame
cmake --build gst_recording_daemon/build

./gst_recording_daemon/build/gst_recording_daemon \
  --device /dev/video2 \
  --camera-host 10.98.32.1 \
  --fps 60 \
  --socket-path /tmp/filmer_recorder_test.sock
```

The daemon probes the capture card and logs the chosen HDMI format. With the current 1080p card, expect something like:

```text
Selected HDMI capture format fourcc=MJPG width=1920 height=1080 fps=30
```

### Smoke Test

In another shell:

```bash
printf 'PING\n' | socat - UNIX-CONNECT:/tmp/filmer_recorder_test.sock
printf 'STATUS\n' | socat - UNIX-CONNECT:/tmp/filmer_recorder_test.sock
```

`STATUS` should show `capture_path:"none"` while idle, camera `ok:true`, and media fields when the Z-CAM USB-C storage is visible.

### Recording Test

In the second shell:

```bash
rm -rf /tmp/filmer_recordings
mkdir -p /tmp/filmer_recordings

printf 'START /tmp/filmer_recordings\n' | socat - UNIX-CONNECT:/tmp/filmer_recorder_test.sock
sleep 5
printf 'STATUS\n' | socat - UNIX-CONNECT:/tmp/filmer_recorder_test.sock
printf 'STOP\n' | socat - UNIX-CONNECT:/tmp/filmer_recorder_test.sock

ls -lh /tmp/filmer_recordings
```

Expected result:

- A non-empty `.mp4` when HDMI capture succeeds.
- `STATUS` reports `capture_path:"hdmi"` while recording.
- The Z-CAM also records locally after `/ctrl/rec?action=start`.

If HDMI capture fails, the daemon attempts SRT fallback and writes a `.mkv`. If both host paths fail, `START` returns an error JSON response.

### Simple Protocol Commands

Non-interactive examples:

```bash
echo "PING" | socat - UNIX-CONNECT:/tmp/filmer_recorder.sock
echo "STATUS" | socat - UNIX-CONNECT:/tmp/filmer_recorder.sock
echo "START /tmp/recordings" | socat - UNIX-CONNECT:/tmp/filmer_recorder.sock
echo "STOP" | socat - UNIX-CONNECT:/tmp/filmer_recorder.sock
```

Interactive:

```bash
socat - UNIX-CONNECT:/tmp/filmer_recorder.sock
```

Then type:

```text
STATUS
START /tmp/recordings
STOP
```
