import json
import socket
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional


RECORDING_STATES = {'STARTING', 'RECORDING'}
STARTABLE_STATES = {'IDLE', 'ERROR'}
BUSY_STATES = {'STOPPING'}


@dataclass(frozen=True)
class DaemonReply:
    ok: bool
    payload: Dict[str, Any]
    raw: str
    error: str = ''

    def message_json(self) -> str:
        if self.raw:
            return self.raw
        return json.dumps(self.payload, separators=(',', ':'))


class GstRecordingDaemonClient:
    def __init__(
        self,
        socket_path: str = '/tmp/filmer_recorder.sock',
        connect_timeout_sec: float = 1.0,
        command_timeout_sec: float = 15.0,
    ) -> None:
        self.socket_path = socket_path
        self.connect_timeout_sec = connect_timeout_sec
        self.command_timeout_sec = command_timeout_sec

    def ping(self) -> DaemonReply:
        return self.command('PING')

    def status(self) -> DaemonReply:
        return self.command('STATUS')

    def start(self, output_dir: str) -> DaemonReply:
        output = str(Path(output_dir).expanduser())
        return self.command(f'START {output}')

    def stop(self) -> DaemonReply:
        return self.command('STOP')

    def command(self, command_line: str) -> DaemonReply:
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
                sock.settimeout(self.connect_timeout_sec)
                sock.connect(self.socket_path)
                sock.settimeout(self.command_timeout_sec)
                sock.sendall(command_line.encode('utf-8') + b'\n')
                return self._read_response(sock)
        except OSError as exc:
            return self._error_reply('daemon_unreachable', str(exc))

    def _read_response(self, sock: socket.socket) -> DaemonReply:
        pending = bytearray()
        while True:
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                return self._error_reply(
                    'daemon_timeout',
                    f'No daemon response within {self.command_timeout_sec:.1f}s',
                )
            if not chunk:
                return self._error_reply(
                    'daemon_disconnected',
                    'Daemon closed the socket before sending a command response',
                )
            pending.extend(chunk)
            while b'\n' in pending:
                line, _, rest = pending.partition(b'\n')
                pending = bytearray(rest)
                raw = line.decode('utf-8', errors='replace').strip()
                if not raw:
                    continue
                reply = self._parse_line(raw)
                if reply.payload.get('type') == 'heartbeat':
                    continue
                return reply

    def _parse_line(self, raw: str) -> DaemonReply:
        try:
            payload = json.loads(raw)
        except json.JSONDecodeError as exc:
            return DaemonReply(
                ok=False,
                payload={
                    'type': 'error',
                    'ok': False,
                    'reason': 'invalid_daemon_json',
                    'detail': str(exc),
                    'raw': raw,
                },
                raw='',
                error='invalid_daemon_json',
            )
        if not isinstance(payload, dict):
            return DaemonReply(
                ok=False,
                payload={
                    'type': 'error',
                    'ok': False,
                    'reason': 'invalid_daemon_json',
                    'detail': 'Daemon JSON response was not an object',
                    'raw': raw,
                },
                raw='',
                error='invalid_daemon_json',
            )

        ok = bool(payload.get('ok', True))
        reason = str(payload.get('reason', '')) if not ok else ''
        return DaemonReply(ok=ok, payload=payload, raw=raw, error=reason)

    def _error_reply(self, reason: str, detail: str) -> DaemonReply:
        payload = {
            'type': 'error',
            'ok': False,
            'reason': reason,
            'detail': detail,
            'socket_path': self.socket_path,
        }
        return DaemonReply(ok=False, payload=payload, raw='', error=reason)


def command_for_toggle(state: Optional[str], output_dir: str) -> Optional[str]:
    normalized = (state or '').upper()
    if normalized in STARTABLE_STATES:
        return f'START {str(Path(output_dir).expanduser())}'
    if normalized in RECORDING_STATES:
        return 'STOP'
    return None


def busy_reason_for_state(state: Optional[str]) -> str:
    normalized = (state or '').upper()
    if normalized in BUSY_STATES:
        return 'daemon_busy_stopping'
    return 'unknown_daemon_state'
