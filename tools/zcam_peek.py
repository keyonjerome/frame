#!/usr/bin/env python3
import argparse
import json
import socket
import struct
import subprocess
import sys
import time
from urllib.request import urlopen, Request
from urllib.error import HTTPError, URLError

def http_get(ip, path, timeout=2):
    url = f"http://{ip}{path}"
    try:
        with urlopen(Request(url), timeout=timeout) as r:
            body = r.read().decode("utf-8", errors="replace")
            try:
                return json.loads(body)
            except json.JSONDecodeError:
                return body
    except Exception as e:
        return {"error": str(e), "url": url}

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="10.98.32.1")
    ap.add_argument("--port", type=int, default=9876)
    args = ap.parse_args()

    print("Camera /info:", http_get(args.ip, "/info"), file=sys.stderr)
    print("Mode:", http_get(args.ip, "/ctrl/mode?action=query"), file=sys.stderr)

    # Optional: keep/claim control session. If another client owns it, /ctrl calls may return 409.
    print("Session:", http_get(args.ip, "/ctrl/session"), file=sys.stderr)

    ffplay = subprocess.Popen([
        "ffplay",
        "-fflags", "nobuffer",
        "-flags", "low_delay",
        "-framedrop",
        "-probesize", "32",
        "-analyzeduration", "0",
        "-f", "h264",
        "-"
    ], stdin=subprocess.PIPE)

    with socket.create_connection((args.ip, args.port), timeout=5) as s:
        s.settimeout(5)
        print(f"Connected to H.264 preview stream at {args.ip}:{args.port}", file=sys.stderr)

        while ffplay.poll() is None:
            # Z CAM doc: send 0x01 to request one frame
            s.sendall(b"\x01")

            # Then receive 4-byte payload length + payload
            hdr = s.recv(4)
            if len(hdr) < 4:
                raise RuntimeError("Short read on length header")

            # Try big-endian first; if absurd, fall back to little-endian.
            n = struct.unpack(">I", hdr)[0]
            if n <= 0 or n > 10_000_000:
                n = struct.unpack("<I", hdr)[0]

            buf = bytearray()
            while len(buf) < n:
                chunk = s.recv(n - len(buf))
                if not chunk:
                    raise RuntimeError("Stream ended")
                buf.extend(chunk)

            ffplay.stdin.write(buf)
            ffplay.stdin.flush()

if __name__ == "__main__":
    main()
