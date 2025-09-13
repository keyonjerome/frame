# Web: Minimal Flask Video Library (Raspberry Pi Ready)

A tiny Flask app optimized for Raspberry Pi or other low-resource devices. It lists videos from a directory, shows size, last modified time, and duration (via ffprobe), and lets users download files via a clean, mobile-friendly UI.

Important: This app is intended for LAN use or a Pi hotspot. If exposing publicly, add authentication and HTTPS (e.g., reverse proxy with TLS).

## TL;DR (Pi Quick Start)
- Put videos in `/videos` (or set `VIDEO_DIR` to your folder).
- Install dependencies:
  ```sh
  sudo apt update
  sudo apt install -y python3-venv python3-pip ffmpeg
  ```
- Clone and run:
  ```sh
  cd ~
  git clone https://github.com/keyonjerome/frame.git
  cd frame/web
  python3 -m venv .venv && source .venv/bin/activate
  pip install --upgrade pip
  pip install flask gunicorn
  export VIDEO_DIR=/videos HOST=0.0.0.0 PORT=1111
  gunicorn -w 2 -b 0.0.0.0:${PORT} app:app
  ```
- Find the Pi’s IP: `hostname -I` (take the first IPv4). Open `http://<PI_IP>:1111/`.
- Create a QR code pointing to that URL (instructions below).

## Features
- Scans a configurable directory for video files (default `../videos`; recommended to set `VIDEO_DIR=/videos`).
- Displays filename, human-readable size, last modified time, and duration.
- Duration extracted via `ffprobe` (ffmpeg). Falls back to `--:--` if unavailable.
- Efficient, streaming downloads (no loading whole file into memory).
- Responsive, minimal UI (Notion-inspired), keyboard-accessible.
- Optional `/health` endpoint for monitoring.

## Requirements
- Python 3.9+
- Flask (pip)
- ffmpeg (for duration via `ffprobe`). The app runs without it, but duration will show `--:--`.

Install ffmpeg (Debian/Raspberry Pi OS):
```sh
a sudo apt update
sudo apt install -y ffmpeg
```

## Configure
- Directory to scan:
  - Default: `VIDEO_DIR=../videos`
  - Recommended on Pi: `VIDEO_DIR=/videos`
- Networking:
  - `HOST=0.0.0.0` (listen on all interfaces)
  - `PORT=1111` (default used in this project)
- Supported extensions: `.mp4, .mov, .mkv, .avi, .webm, .m4v`.
- If the directory is missing or empty, the page shows: `No Video's found in <path>`.

## Run (simple)
```sh
cd frame/web
python3 -m venv .venv && source .venv/bin/activate
pip install --upgrade pip
pip install flask
export VIDEO_DIR=/videos HOST=0.0.0.0 PORT=1111
python3 app.py
```
Visit: `http://<PI_IP>:1111/`.

## Run with Gunicorn (recommended)
```sh
cd frame/web
source .venv/bin/activate  # if created above
pip install gunicorn
export VIDEO_DIR=/videos HOST=0.0.0.0 PORT=1111
exec gunicorn -w 2 -b 0.0.0.0:${PORT} app:app
```
Notes:
- `-w 2` starts two worker processes (good starting point for a Pi). You can also add `--threads 2`.
- Use `--access-logfile -` to log requests to stdout if desired.

## Systemd service (auto-start on boot)
Create `/etc/systemd/system/video-library.service` (adjust paths/user):
```ini
[Unit]
Description=Flask Video Library (Gunicorn)
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/frame/web
Environment=VIDEO_DIR=/videos
Environment=PORT=1111
ExecStart=/home/pi/frame/web/.venv/bin/gunicorn -w 2 -b 0.0.0.0:${PORT} app:app
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```
Enable and start:
```sh
sudo systemctl daemon-reload
sudo systemctl enable --now video-library
sudo systemctl status video-library --no-pager
```
Logs:
```sh
journalctl -u video-library -e --no-pager
```

## Find the Pi’s IP address
- `hostname -I`  (first IPv4 is typical)
- or `ip -4 addr show wlan0 | grep -oP '(?<=inet\s)\d+(\.\d+){3}'`
Use it in `http://<PI_IP>:1111/`.

## Create a QR code to the website
The app does NOT generate QR codes. Make one manually using either a phone/PC or on the Pi.

Option A: On the Pi (CLI)
```sh
sudo apt install -y qrencode
URL="http://$(hostname -I | awk '{print $1}'):1111/"
# Save to PNG (view or print)
qrencode -o ~/video-library-qr.png "$URL"
# Or show in terminal (monochrome)
qrencode -t ANSIUTF8 "$URL"
```

Option B: On macOS/iOS/Android
- Use any QR code app or website.
- Enter the Pi URL, e.g., `http://192.168.1.23:1111/`.
- Print or place the QR near the Pi.

## Raspberry Pi Hotspot (optional, high level)
- Install and enable `hostapd` (Wi‑Fi AP) and `dnsmasq` (DHCP).
- Set a static IP on `wlan0` (e.g., `192.168.4.1`).
- Configure `hostapd` with your SSID/password/country.
- Configure `dnsmasq` to serve DHCP in `192.168.4.0/24`.
- Start services and run this app on `0.0.0.0:1111`.
- Clients connect to your SSID and open `http://192.168.4.1:1111/`.

## Troubleshooting
- Durations show `--:--`: Install `ffmpeg` or ensure `ffprobe` is in PATH.
- No videos listed: Ensure `VIDEO_DIR` is correct and readable. The page shows `No Video's found in <path>` when empty.
- Cannot access from phone: Verify Pi IP, port `1111`, and that the service is listening on `0.0.0.0`.
- Slow downloads: Lower workers/threads on very small Pis; use wired Ethernet if possible.

## Security Notes
- Designed for LAN or AP use. If exposing to the Internet: add authentication, enable HTTPS (reverse proxy with TLS), and keep ffmpeg/Flask updated.

## Project Layout
```
web/
  app.py
  templates/
    index.html
  README.md
```
