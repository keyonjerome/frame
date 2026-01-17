from fractions import Fraction
from pathlib import Path
from typing import Optional

try:
    import gi

    gi.require_version('Gst', '1.0')
    from gi.repository import Gst
except (ImportError, ValueError):
    Gst = None


_GST_INITIALIZED = False


def _ensure_gst(logger) -> bool:
    global _GST_INITIALIZED
    if Gst is None:
        logger.error(
            'GStreamer Python bindings not available. Install python3-gi and '
            'gir1.2-gstreamer-1.0.'
        )
        return False
    if not _GST_INITIALIZED:
        Gst.init(None)
        _GST_INITIALIZED = True
    return True


def _element_available(name: str) -> bool:
    return Gst is not None and Gst.ElementFactory.find(name) is not None


class GstreamerWriter:
    def __init__(
        self,
        output_path: Path,
        width: int,
        height: int,
        fps: float,
        logger,
        encoder: str = '',
        enable_audio: bool = True,
        audio_device: str = '',
        audio_encoder: str = '',
    ) -> None:
        self._logger = logger
        self._pipeline = None
        self._appsrc = None
        self._bus = None
        self._using_audio = False
        self._timestamp = 0
        self._duration = 0
        self._open(
            output_path,
            width,
            height,
            fps,
            encoder,
            enable_audio,
            audio_device,
            audio_encoder,
        )

    def _select_encoder(self, requested: str) -> Optional[str]:
        candidates = []
        if requested:
            candidates = [requested]
        else:
            candidates = [
                'x264enc speed-preset=ultrafast tune=zerolatency',
                'openh264enc',
                'avenc_h264',
            ]
        for candidate in candidates:
            name = candidate.split()[0]
            if _element_available(name):
                return candidate
        return None

    def _select_muxer(self) -> Optional[str]:
        for muxer in ('mp4mux', 'qtmux'):
            if _element_available(muxer):
                return muxer
        return None

    def _select_audio_encoder(self, requested: str) -> Optional[str]:
        candidates = []
        if requested:
            candidates = [requested]
        else:
            candidates = ['avenc_aac', 'voaacenc', 'faac']
        for candidate in candidates:
            if _element_available(candidate):
                return candidate
        return None

    def _select_audio_source(self, device: str) -> Optional[str]:
        if device:
            if _element_available('alsasrc'):
                return f'alsasrc device={device}'
            if _element_available('pulsesrc'):
                return f'pulsesrc device={device}'
            return None
        if _element_available('autoaudiosrc'):
            return 'autoaudiosrc'
        if _element_available('pulsesrc'):
            return 'pulsesrc'
        if _element_available('alsasrc'):
            return 'alsasrc'
        return None

    def _open(
        self,
        output_path: Path,
        width: int,
        height: int,
        fps: float,
        encoder: str,
        enable_audio: bool,
        audio_device: str,
        audio_encoder: str,
    ) -> None:
        if not _ensure_gst(self._logger):
            return

        if enable_audio:
            if self._try_open(
                output_path,
                width,
                height,
                fps,
                encoder,
                True,
                audio_device,
                audio_encoder,
                'with audio',
            ):
                self._using_audio = True
                return
            self._logger.warn(
                'Audio unavailable; falling back to video-only recording.'
            )

        if self._try_open(
            output_path,
            width,
            height,
            fps,
            encoder,
            False,
            audio_device,
            audio_encoder,
            'video-only',
        ):
            self._using_audio = False

    def _try_open(
        self,
        output_path: Path,
        width: int,
        height: int,
        fps: float,
        encoder: str,
        enable_audio: bool,
        audio_device: str,
        audio_encoder: str,
        mode_label: str,
    ) -> bool:
        encoder_desc = self._select_encoder(encoder)
        if encoder_desc is None:
            self._logger.error(
                'No supported GStreamer H.264 encoder found. Install '
                'x264enc, openh264enc, or avenc_h264.'
            )
            return False

        muxer = self._select_muxer()
        if muxer is None:
            self._logger.error('No MP4 muxer available (mp4mux or qtmux).')
            return False

        audio_desc = ''
        if enable_audio:
            audio_src = self._select_audio_source(audio_device)
            if audio_src is None:
                self._logger.warn('No audio source available for GStreamer.')
                return False
            audio_enc = self._select_audio_encoder(audio_encoder)
            if audio_enc is None:
                self._logger.warn(
                    'No supported GStreamer AAC encoder found. Install avenc_aac, '
                    'voaacenc, or faac.'
                )
                return False
            audio_desc = (
                f'{audio_src} '
                '! audioconvert '
                '! audioresample '
                f'! {audio_enc} '
                '! queue '
                '! mux. '
            )

        pipeline_desc = (
            'appsrc name=src is-live=true block=false format=time do-timestamp=true '
            '! videoconvert '
            '! video/x-raw,format=I420 '
            f'! {encoder_desc} '
            '! h264parse '
            '! queue '
            '! mux. '
            f'{audio_desc}'
            f'{muxer} name=mux '
            '! filesink name=sink'
        )

        try:
            pipeline = Gst.parse_launch(pipeline_desc)
        except Exception as exc:  # noqa: BLE001
            self._logger.error(
                f'Failed to build GStreamer pipeline ({mode_label}): {exc}'
            )
            return False

        appsrc = pipeline.get_by_name('src')
        sink = pipeline.get_by_name('sink')
        mux = pipeline.get_by_name('mux')
        if appsrc is None or sink is None:
            self._logger.error('GStreamer pipeline missing appsrc/filesink.')
            pipeline.set_state(Gst.State.NULL)
            return False

        sink.set_property('location', str(output_path))
        if mux is not None and mux.find_property('faststart'):
            mux.set_property('faststart', True)

        fps_value = fps if fps > 0.0 else 30.0
        fps_fraction = Fraction(fps_value).limit_denominator(1000)
        caps = Gst.Caps.from_string(
            'video/x-raw,format=BGR,'
            f'width={width},height={height},'
            f'framerate={fps_fraction.numerator}/{fps_fraction.denominator}'
        )
        appsrc.set_property('caps', caps)
        appsrc.set_property('is-live', True)
        appsrc.set_property('block', False)
        appsrc.set_property('format', Gst.Format.TIME)
        appsrc.set_property('do-timestamp', True)

        self._duration = int(
            1e9 * fps_fraction.denominator / fps_fraction.numerator
        )
        self._timestamp = 0

        ret = pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            self._logger.error(
                f'GStreamer pipeline failed to start ({mode_label}).'
            )
            pipeline.set_state(Gst.State.NULL)
            return False
        if ret == Gst.StateChangeReturn.ASYNC:
            ret, _, _ = pipeline.get_state(5 * Gst.SECOND)
            if ret == Gst.StateChangeReturn.FAILURE:
                self._logger.error(
                    f'GStreamer pipeline failed to reach PLAYING ({mode_label}).'
                )
                pipeline.set_state(Gst.State.NULL)
                return False

        self._pipeline = pipeline
        self._appsrc = appsrc
        self._bus = pipeline.get_bus()

        self._logger.info(
            f'GStreamer recording ({mode_label}) with '
            f'{encoder_desc.split()[0]} -> {output_path}'
        )
        return True

    def is_open(self) -> bool:
        return self._pipeline is not None and self._appsrc is not None

    def _check_bus(self) -> bool:
        if self._bus is None:
            return True
        msg = self._bus.timed_pop_filtered(
            0, Gst.MessageType.ERROR | Gst.MessageType.EOS
        )
        if msg is None:
            return True
        if msg.type == Gst.MessageType.ERROR:
            err, dbg = msg.parse_error()
            detail = f' ({dbg})' if dbg else ''
            self._logger.error(f'GStreamer error: {err.message}{detail}')
        elif msg.type == Gst.MessageType.EOS:
            self._logger.warn('GStreamer reached end-of-stream.')
        return False

    def write(self, frame) -> bool:
        if self._pipeline is None or self._appsrc is None:
            return False
        if not self._check_bus():
            return False
        if not frame.flags['C_CONTIGUOUS']:
            frame = frame.copy()
        buffer = Gst.Buffer.new_allocate(None, frame.nbytes, None)
        buffer.fill(0, frame.tobytes())
        buffer.pts = self._timestamp
        buffer.dts = self._timestamp
        buffer.duration = self._duration
        self._timestamp += self._duration
        result = self._appsrc.emit('push-buffer', buffer)
        if result != Gst.FlowReturn.OK:
            self._logger.error('GStreamer failed to push buffer.')
            return False
        return self._check_bus()

    def close(self) -> None:
        if self._pipeline is None:
            return
        try:
            if self._appsrc is not None:
                self._appsrc.emit('end-of-stream')
            if self._bus is not None:
                msg = self._bus.timed_pop_filtered(
                    2 * Gst.SECOND, Gst.MessageType.ERROR | Gst.MessageType.EOS
                )
                if msg and msg.type == Gst.MessageType.ERROR:
                    err, dbg = msg.parse_error()
                    detail = f' ({dbg})' if dbg else ''
                    self._logger.error(f'GStreamer error: {err.message}{detail}')
        finally:
            self._pipeline.set_state(Gst.State.NULL)
            self._pipeline = None
            self._appsrc = None
            self._bus = None
