import subprocess
import threading
from pathlib import Path
from typing import Optional


class FfmpegWriter:
    def __init__(
        self,
        output_path: Path,
        width: int,
        height: int,
        fps: float,
        ffmpeg_path: str,
        log_stderr: bool,
        logger,
    ) -> None:
        self._logger = logger
        self._process: Optional[subprocess.Popen] = None
        self._stderr_thread: Optional[threading.Thread] = None
        self._log_stderr = log_stderr
        self._open(output_path, width, height, fps, ffmpeg_path)

    def _open(
        self,
        output_path: Path,
        width: int,
        height: int,
        fps: float,
        ffmpeg_path: str,
    ) -> None:
        cmd = [
            ffmpeg_path,
            '-hide_banner',
            '-loglevel',
            'error',
            '-y',
            '-f',
            'rawvideo',
            '-pix_fmt',
            'bgr24',
            '-s',
            f'{width}x{height}',
            '-r',
            f'{fps:.2f}',
            '-i',
            '-',
            '-vf',
            'scale=ceil(iw/2)*2:ceil(ih/2)*2',
            '-an',
            '-c:v',
            'mpeg4',
            '-qscale:v',
            '5',
            '-pix_fmt',
            'yuv420p',
            str(output_path),
        ]
        stderr = subprocess.PIPE if self._log_stderr else subprocess.DEVNULL
        try:
            self._process = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stderr=stderr,
            )
            if self._log_stderr and self._process.stderr:
                self._stderr_thread = threading.Thread(
                    target=self._drain_stderr,
                    args=(self._process.stderr,),
                    daemon=True,
                )
                self._stderr_thread.start()
        except FileNotFoundError:
            self._logger.error(
                f'ffmpeg not found at "{ffmpeg_path}". Install ffmpeg or update ffmpeg_path.'
            )
            self._process = None
        except Exception as exc:  # noqa: BLE001
            self._logger.error(f'Failed to start ffmpeg: {exc}')
            self._process = None

    def _drain_stderr(self, stream) -> None:
        for raw in iter(stream.readline, b''):
            line = raw.decode('utf-8', errors='replace').strip()
            if line:
                self._logger.warn(f'ffmpeg: {line}')
        try:
            stream.close()
        except Exception:
            pass

    def is_open(self) -> bool:
        return self._process is not None

    def write(self, frame) -> bool:
        if self._process is None or self._process.stdin is None:
            return False
        if self._process.poll() is not None:
            self._logger.error('ffmpeg exited unexpectedly during recording.')
            return False
        if not frame.flags['C_CONTIGUOUS']:
            frame = frame.copy()
        try:
            self._process.stdin.write(frame.tobytes())
        except BrokenPipeError:
            self._logger.error('ffmpeg pipe closed while recording.')
            return False
        return True

    def close(self) -> None:
        if self._process is None:
            return
        try:
            if self._process.stdin:
                self._process.stdin.close()
        except Exception:
            pass
        try:
            if self._process.stderr:
                self._process.stderr.close()
        except Exception:
            pass
        try:
            self._process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            self._process.terminate()
        self._process = None
