from __future__ import annotations

import os
import mimetypes
import subprocess
from datetime import datetime
from pathlib import Path
from typing import List, Dict, Optional

from flask import Flask, render_template, url_for, abort, send_file, Response
from werkzeug.utils import safe_join

# ----------------------------
# Configuration
# ----------------------------
# Default directory to scan for videos. You can also override via env var VIDEO_DIR.
VIDEO_DIR: str = os.environ.get("VIDEO_DIR", "../videos")
# Allowed video extensions (lowercase)
VIDEO_EXTS = {".mp4", ".mov", ".mkv", ".avi", ".webm", ".m4v"}

app = Flask(__name__)


# ----------------------------
# Helpers
# ----------------------------

def human_size(num_bytes: int) -> str:
    """Return human-readable size (e.g., 932 MB, 1.8 GB) with one decimal for GB."""
    if num_bytes is None:
        return "-"
    units = ["B", "KB", "MB", "GB", "TB"]
    size = float(num_bytes)
    for unit in units:
        if size < 1024.0 or unit == units[-1]:
            if unit in ("GB", "TB"):
                return f"{size:.1f} {unit}"
            if unit == "MB":
                # round to whole MB for cleaner display
                return f"{int(round(size))} MB"
            return f"{int(size)} {unit}"
        size /= 1024.0


def human_time(ts: float) -> str:
    """Format epoch seconds to local time YYYY-MM-DD HH:MM."""
    try:
        return datetime.fromtimestamp(ts).strftime("%Y-%m-%d %H:%M")
    except Exception:
        return "-"


def format_duration_seconds(seconds: Optional[float]) -> str:
    """Convert seconds to MM:SS or H:MM:SS. Return "--:--" if None."""
    if not seconds or seconds <= 0:
        return "--:--"
    total = int(round(seconds))
    h, rem = divmod(total, 3600)
    m, s = divmod(rem, 60)
    if h:
        return f"{h}:{m:02d}:{s:02d}"
    return f"{m:02d}:{s:02d}"


def get_duration_via_ffprobe(path: Path) -> Optional[float]:
    """Use ffprobe to extract duration in seconds. Returns None on failure.

    Command used:
      ffprobe -v error -show_entries format=duration -of default=noprint_wrappers=1:nokey=1 <file>
    """
    try:
        # Run without shell for safety; capture stdout only
        result = subprocess.run(
            [
                "ffprobe",
                "-v",
                "error",
                "-show_entries",
                "format=duration",
                "-of",
                "default=noprint_wrappers=1:nokey=1",
                str(path),
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            check=False,
            text=True,
            timeout=5.0,
        )
        if result.returncode != 0:
            return None
        out = result.stdout.strip()
        if not out:
            return None
        # ffprobe returns a float in seconds
        return float(out)
    except (FileNotFoundError, ValueError, subprocess.TimeoutExpired):
        # ffprobe not installed or parse error
        return None


def list_videos() -> List[Dict[str, str]]:
    """Scan VIDEO_DIR for video files. Returns an empty list if none found."""
    base = Path(VIDEO_DIR)
    items: List[Dict[str, str]] = []

    if base.exists() and base.is_dir():
        for p in sorted(base.iterdir()):
            if not p.is_file():
                continue
            if p.suffix.lower() not in VIDEO_EXTS:
                continue
            try:
                stat = p.stat()
            except OSError:
                continue
            size_str = human_size(stat.st_size)
            modified_str = human_time(stat.st_mtime)
            dur_sec = get_duration_via_ffprobe(p)
            duration_str = format_duration_seconds(dur_sec)
            items.append(
                {
                    "name": p.name,
                    "size_str": size_str,
                    "modified_str": modified_str,
                    "duration_str": duration_str,
                }
            )

    return items


# ----------------------------
# Routes
# ----------------------------

@app.get("/")
def index() -> str:
    videos = list_videos()
    # Build absolute base URL for copy-link convenience
    return render_template(
        "index.html",
        videos=videos,
        video_dir=VIDEO_DIR,
    )


@app.get("/download/<path:filename>")
def download(filename: str):
    # Sanitize path and ensure it's inside VIDEO_DIR
    safe_path = safe_join(VIDEO_DIR, filename)
    if not safe_path:
        abort(404)
    file_path = Path(safe_path)
    if not file_path.exists() or not file_path.is_file():
        abort(404)

    mime, _ = mimetypes.guess_type(str(file_path))
    mime = mime or "application/octet-stream"

    # send_file will stream from disk; conditional=True enables range/If-Modified-Since
    resp = send_file(
        path_or_file=str(file_path),
        mimetype=mime,
        as_attachment=True,
        download_name=file_path.name,
        conditional=True,
        etag=True,
        last_modified=file_path.stat().st_mtime,
    )
    # Security hardening
    resp.headers.setdefault("X-Content-Type-Options", "nosniff")
    resp.headers.setdefault("Cache-Control", "private, max-age=3600, must-revalidate")
    return resp


@app.get("/health")
def health() -> Response:
    return Response("OK", mimetype="text/plain")


# ----------------------------
# Entrypoint
# ----------------------------
if __name__ == "__main__":
    # Bind to all interfaces by default (useful for LAN / AP scenarios)
    host = os.environ.get("HOST", "0.0.0.0")
    port = int(os.environ.get("PORT", "1111"))
    # Production tip: use gunicorn for better performance/stability.
    app.run(host=host, port=port, threaded=True)
