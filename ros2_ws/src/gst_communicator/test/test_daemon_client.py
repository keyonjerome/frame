import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from gst_communicator.daemon_client import (  # noqa: E402
    GstRecordingDaemonClient,
    busy_reason_for_state,
    command_for_toggle,
)


class FakeSocket:
    def __init__(self, responses=None, connect_error=None):
        self.responses = list(responses or [])
        self.connect_error = connect_error
        self.commands = []
        self.timeouts = []
        self.connected_path = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        return None

    def settimeout(self, timeout):
        self.timeouts.append(timeout)

    def connect(self, socket_path):
        if self.connect_error is not None:
            raise self.connect_error
        self.connected_path = socket_path

    def sendall(self, data):
        self.commands.append(data.decode('utf-8').strip())

    def recv(self, size):
        del size
        if self.responses:
            return self.responses.pop(0)
        return b''


def install_fake_socket(monkeypatch, fake_socket):
    import socket

    monkeypatch.setattr(socket, 'socket', lambda *args, **kwargs: fake_socket)


def test_status_returns_daemon_json(monkeypatch):
    fake_socket = FakeSocket([
        b'{"type":"status","state":"IDLE","healthy":true}\n',
    ])
    install_fake_socket(monkeypatch, fake_socket)
    reply = GstRecordingDaemonClient('/tmp/daemon.sock').status()

    assert fake_socket.connected_path == '/tmp/daemon.sock'
    assert fake_socket.commands == ['STATUS']
    assert reply.ok is True
    assert reply.payload['state'] == 'IDLE'
    assert reply.message_json() == '{"type":"status","state":"IDLE","healthy":true}'


def test_ignores_heartbeat_before_command_response(monkeypatch):
    fake_socket = FakeSocket([
        b'{"type":"heartbeat","state":"RECORDING","healthy":true}\n',
        b'{"type":"status","state":"RECORDING","healthy":true}\n',
    ])
    install_fake_socket(monkeypatch, fake_socket)
    reply = GstRecordingDaemonClient('/tmp/daemon.sock').status()

    assert reply.ok is True
    assert reply.payload['type'] == 'status'
    assert reply.payload['state'] == 'RECORDING'


def test_socket_failure_returns_error_json(monkeypatch):
    fake_socket = FakeSocket(connect_error=OSError('missing socket'))
    install_fake_socket(monkeypatch, fake_socket)
    reply = GstRecordingDaemonClient('/tmp/missing.sock', connect_timeout_sec=0.01).status()

    assert reply.ok is False
    assert reply.payload['type'] == 'error'
    assert reply.payload['reason'] == 'daemon_unreachable'
    assert reply.payload['socket_path'] == '/tmp/missing.sock'


def test_start_and_stop_commands(monkeypatch):
    start_socket = FakeSocket([
        b'{"type":"start","ok":true,"state":"RECORDING"}\n',
    ])
    install_fake_socket(monkeypatch, start_socket)
    reply = GstRecordingDaemonClient('/tmp/start.sock').start('/tmp/out')

    assert start_socket.commands == ['START /tmp/out']
    assert reply.ok is True
    assert reply.payload['state'] == 'RECORDING'

    stop_socket = FakeSocket([
        b'{"type":"stop","ok":true,"state":"IDLE"}\n',
    ])
    install_fake_socket(monkeypatch, stop_socket)
    reply = GstRecordingDaemonClient('/tmp/stop.sock').stop()

    assert stop_socket.commands == ['STOP']
    assert reply.ok is True
    assert reply.payload['state'] == 'IDLE'


def test_toggle_command_decisions():
    assert command_for_toggle('IDLE', '/tmp/out') == 'START /tmp/out'
    assert command_for_toggle('ERROR', '/tmp/out') == 'START /tmp/out'
    assert command_for_toggle('RECORDING', '/tmp/out') == 'STOP'
    assert command_for_toggle('STARTING', '/tmp/out') == 'STOP'
    assert command_for_toggle('STOPPING', '/tmp/out') is None
    assert busy_reason_for_state('STOPPING') == 'daemon_busy_stopping'
