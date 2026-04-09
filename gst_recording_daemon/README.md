# gst_recording_daemon

`gst_recording_daemon` is a standalone C++17 Linux daemon for recording a V4L2 HDMI-to-USB capture device directly to MP4 using GStreamer. It is intentionally minimal and does not use ROS 2.

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
  gstreamer1.0-libav \
  socat
```

`h264parse` and `mp4mux` must be available at runtime.

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
./gst_recording_daemon --device /dev/video0 --fps 60
```

Optional flags:

```bash
./gst_recording_daemon --device /dev/video2 --fps 30 --socket-path /tmp/filmer_recorder.sock
```

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
- The daemon creates a timestamped file like `recording_20250101_120000.mp4` inside that directory.
- Only one client is supported at a time. Extra clients are rejected while one is connected.
- MP4 finalization depends on graceful stop via EOS.

## Quick Tests

Non-interactive:

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
