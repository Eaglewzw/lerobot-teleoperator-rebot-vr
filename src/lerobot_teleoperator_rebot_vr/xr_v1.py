"""XRoboToolkit V1 stream decoding and latest-only TCP Tracking source."""

from __future__ import annotations

import json
import socket
import struct
import threading
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass, replace

from .tracking import (
    ControllerSample,
    LatestSampleBuffer,
    TrackingSampleError,
    normalize_controller_side,
    parse_controller_sample,
)


SEND_PACKET_HEAD = 0x3F
RECEIVE_PACKET_HEAD = 0xCF
PACKET_END = 0xA5
MAX_BODY_SIZE = 1024 * 1024

CMD_CONNECT = 0x19
CMD_VERSION = 0x6C
CMD_HEARTBEAT = 0x23
CMD_FUNCTION = 0x6D
CMD_FUNCTION_FROM_PC = 0x5F
CMD_CUSTOM_TO_PC = 0x72

CMD_NAMES = {
    CMD_CONNECT: "CONNECT",
    CMD_VERSION: "VERSION",
    CMD_HEARTBEAT: "HEARTBEAT",
    CMD_FUNCTION: "FUNCTION",
    CMD_FUNCTION_FROM_PC: "FUNCTION_FROM_PC",
    CMD_CUSTOM_TO_PC: "CUSTOM_DATA",
}


class PacketParser:
    """Encoder/decoder for one complete little-endian V1 packet."""

    @staticmethod
    def unpack(data: bytes) -> dict[str, object] | None:
        if len(data) < 15 or data[0] not in (SEND_PACKET_HEAD, RECEIVE_PACKET_HEAD):
            return None
        body_len = struct.unpack_from("<i", data, 2)[0]
        if body_len < 0 or body_len > MAX_BODY_SIZE:
            return None
        packet_len = 15 + body_len
        if len(data) < packet_len or data[packet_len - 1] != PACKET_END:
            return None
        command = data[1]
        body = data[6 : 6 + body_len]
        timestamp = struct.unpack_from("<q", data, 6 + body_len)[0]
        return {
            "cmd": command,
            "cmd_name": CMD_NAMES.get(command, f"UNKNOWN(0x{command:02X})"),
            "body": body,
            "body_str": body.decode("utf-8", errors="replace"),
            "timestamp": timestamp,
            "direction": "SEND" if data[0] == SEND_PACKET_HEAD else "RECV",
        }

    @staticmethod
    def pack(command: int, body: str | bytes) -> bytes:
        body_bytes = body.encode("utf-8") if isinstance(body, str) else bytes(body)
        if len(body_bytes) > MAX_BODY_SIZE:
            raise ValueError(f"packet body is too large: {len(body_bytes)}")
        if not 0 <= int(command) <= 0xFF:
            raise ValueError("command must fit in one byte")
        return b"".join(
            (
                struct.pack("<BBi", RECEIVE_PACKET_HEAD, int(command), len(body_bytes)),
                body_bytes,
                struct.pack("<qB", int(time.time() * 1000), PACKET_END),
            )
        )


class PacketStreamDecoder:
    """Recover V1 packets from fragmented, coalesced, or corrupt TCP bytes."""

    def __init__(self, on_warning: Callable[[str], None] | None = None) -> None:
        self._buffer = bytearray()
        self._on_warning = on_warning or (lambda _message: None)

    @property
    def buffered_bytes(self) -> int:
        return len(self._buffer)

    def reset(self) -> None:
        self._buffer.clear()

    def feed(self, data: bytes) -> list[dict[str, object]]:
        if data:
            self._buffer.extend(data)
        packets: list[dict[str, object]] = []
        while len(self._buffer) >= 15:
            head_index = self._find_head()
            if head_index is None:
                skipped = len(self._buffer)
                self._buffer.clear()
                self._on_warning(f"skipped {skipped} invalid bytes")
                break
            if head_index:
                del self._buffer[:head_index]
                self._on_warning(f"skipped {head_index} invalid bytes")
            if len(self._buffer) < 15:
                break

            body_len = struct.unpack_from("<i", self._buffer, 2)[0]
            if body_len < 0 or body_len > MAX_BODY_SIZE:
                self._on_warning(f"invalid body length {body_len}")
                del self._buffer[0]
                continue
            packet_len = 15 + body_len
            if len(self._buffer) < packet_len:
                break
            packet = PacketParser.unpack(bytes(self._buffer[:packet_len]))
            if packet is None:
                self._on_warning("packet validation failed")
                del self._buffer[0]
                continue
            del self._buffer[:packet_len]
            packets.append(packet)
        return packets

    def _find_head(self) -> int | None:
        for index, value in enumerate(self._buffer):
            if value in (SEND_PACKET_HEAD, RECEIVE_PACKET_HEAD):
                return index
        return None


class TrackingDecoder:
    """Decode the two JSON layers used by a V1 function message."""

    @staticmethod
    def decode_function(packet: Mapping[str, object]) -> tuple[str, object] | None:
        outer = TrackingDecoder.decode_json_object(packet.get("body_str"))
        if outer is None:
            return None
        function_name = outer.get("functionName")
        if not isinstance(function_name, str):
            return None
        value = outer.get("value")
        if isinstance(value, str):
            try:
                value = json.loads(value)
            except (json.JSONDecodeError, TypeError):
                return None
        return function_name, value

    @staticmethod
    def decode_tracking(packet: Mapping[str, object]) -> dict[str, object] | None:
        function = TrackingDecoder.decode_function(packet)
        if function is None:
            return None
        function_name, value = function
        return value if function_name == "Tracking" and isinstance(value, dict) else None

    @staticmethod
    def decode_json_object(value: object) -> dict[str, object] | None:
        if not isinstance(value, str):
            return None
        try:
            decoded = json.loads(value)
        except (json.JSONDecodeError, TypeError):
            return None
        return decoded if isinstance(decoded, dict) else None


BytesCallback = Callable[[bytes], None]
StatusCallback = Callable[[str], None]
SampleCallback = Callable[[ControllerSample, Mapping[str, object]], None]


class TcpReceiver:
    """Single-client TCP server responsible only for socket lifecycle."""

    def __init__(
        self,
        host: str,
        port: int,
        on_bytes: BytesCallback,
        *,
        on_listen: Callable[[], None] | None = None,
        on_connect: Callable[[tuple[str, int]], None] | None = None,
        on_disconnect: Callable[[], None] | None = None,
        on_error: StatusCallback | None = None,
    ) -> None:
        self.host = host
        self.port = int(port)
        self._on_bytes = on_bytes
        self._on_listen = on_listen or (lambda: None)
        self._on_connect = on_connect or (lambda _address: None)
        self._on_disconnect = on_disconnect or (lambda: None)
        self._on_error = on_error or (lambda _message: None)
        self._server_socket: socket.socket | None = None
        self._client_socket: socket.socket | None = None
        self._socket_lock = threading.Lock()
        self._running = False

    @property
    def running(self) -> bool:
        return self._running

    def serve_forever(self) -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.host, self.port))
        server.listen(1)
        server.settimeout(0.5)
        with self._socket_lock:
            self._server_socket = server
            self._running = True
        self._on_listen()
        try:
            while self._running:
                try:
                    client, address = server.accept()
                except socket.timeout:
                    continue
                except OSError as exc:
                    if self._running:
                        self._on_error(f"accept failed: {exc}")
                    break
                with self._socket_lock:
                    self._client_socket = client
                self._on_connect(address)
                self._receive_client(client)
        finally:
            self.stop()

    def stop(self) -> None:
        self._running = False
        with self._socket_lock:
            client = self._client_socket
            server = self._server_socket
            self._client_socket = None
            self._server_socket = None
        for sock in (client, server):
            if sock is None:
                continue
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass

    def _receive_client(self, client: socket.socket) -> None:
        client.settimeout(0.5)
        try:
            while self._running:
                try:
                    data = client.recv(65536)
                except socket.timeout:
                    continue
                except (ConnectionResetError, ConnectionAbortedError, OSError) as exc:
                    if self._running:
                        self._on_error(f"connection interrupted: {exc}")
                    break
                if not data:
                    break
                try:
                    self._on_bytes(data)
                except Exception as exc:
                    self._on_error(f"received data handler failed: {exc}")
                    break
        finally:
            with self._socket_lock:
                if self._client_socket is client:
                    self._client_socket = None
            try:
                client.close()
            except OSError:
                pass
            self._on_disconnect()


@dataclass(frozen=True)
class TrackingSourceStats:
    connected: bool
    stream_epoch: int
    packet_count: int
    tracking_frame_count: int
    invalid_frame_count: int
    decode_warning_count: int
    last_error: str | None


class V1TrackingSource:
    """Background V1 server publishing only the latest valid hand sample."""

    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = 63901,
        *,
        side: str = "right",
        on_status: StatusCallback | None = None,
        on_sample: SampleCallback | None = None,
    ) -> None:
        if not 1 <= int(port) <= 65535:
            raise ValueError("port must be in [1, 65535]")
        self.host = str(host)
        self.port = int(port)
        self.side = normalize_controller_side(side)
        self._on_status = on_status or (lambda _message: None)
        self._on_sample = on_sample
        self._buffer = LatestSampleBuffer()
        self._decoder = PacketStreamDecoder(self._on_decode_warning)
        self._lock = threading.Lock()
        self._connected = False
        self._stream_epoch = 0
        self._last_tracking_timestamp_ns = 0
        self._packet_count = 0
        self._tracking_frame_count = 0
        self._invalid_frame_count = 0
        self._decode_warning_count = 0
        self._last_error: str | None = None
        self._ready = threading.Event()
        self._startup_error: BaseException | None = None
        self._thread: threading.Thread | None = None
        self._receiver = TcpReceiver(
            host=self.host,
            port=self.port,
            on_bytes=self.feed_bytes,
            on_listen=self._on_listen,
            on_connect=self._on_connect,
            on_disconnect=self._on_disconnect,
            on_error=self._on_receive_error,
        )

    def start(self, timeout: float = 3.0) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._ready.clear()
        self._startup_error = None
        self._thread = threading.Thread(
            target=self._run, name="rebot-vr-v1", daemon=True
        )
        self._thread.start()
        if not self._ready.wait(timeout=max(0.1, float(timeout))):
            self.stop()
            raise TimeoutError(f"V1 server did not start within {timeout:.1f}s")
        if self._startup_error is not None:
            error = self._startup_error
            self.stop()
            raise RuntimeError(
                f"failed to listen on {self.host}:{self.port}: {error}"
            ) from error

    def stop(self) -> None:
        self._receiver.stop()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        self._thread = None
        with self._lock:
            self._connected = False
        self._buffer.clear()

    @property
    def running(self) -> bool:
        thread = self._thread
        return bool(thread is not None and thread.is_alive())

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._connected

    def latest_sample(self) -> ControllerSample | None:
        sample, _ = self._buffer.latest()
        return sample

    def latest(self) -> tuple[ControllerSample | None, int]:
        return self._buffer.latest()

    def stats(self) -> TrackingSourceStats:
        with self._lock:
            return TrackingSourceStats(
                connected=self._connected,
                stream_epoch=self._stream_epoch,
                packet_count=self._packet_count,
                tracking_frame_count=self._tracking_frame_count,
                invalid_frame_count=self._invalid_frame_count,
                decode_warning_count=self._decode_warning_count,
                last_error=self._last_error,
            )

    def feed_bytes(
        self, data: bytes, *, received_monotonic_ns: int | None = None
    ) -> None:
        received_ns = (
            time.monotonic_ns()
            if received_monotonic_ns is None
            else int(received_monotonic_ns)
        )
        for packet in self._decoder.feed(data):
            self._handle_packet(packet, received_ns)

    def _run(self) -> None:
        try:
            self._receiver.serve_forever()
        except BaseException as exc:
            self._startup_error = exc
            with self._lock:
                self._last_error = str(exc)
            self._ready.set()
        finally:
            self._ready.set()

    def _on_listen(self) -> None:
        self._ready.set()
        self._on_status(f"listening on {self.host}:{self.port}")

    def _on_connect(self, address: tuple[str, int]) -> None:
        self._decoder.reset()
        self._buffer.clear()
        with self._lock:
            self._connected = True
            self._stream_epoch += 1
            self._last_tracking_timestamp_ns = 0
        self._on_status(f"headset connected: {address[0]}:{address[1]}")

    def _on_disconnect(self) -> None:
        self._buffer.clear()
        with self._lock:
            was_connected = self._connected
            self._connected = False
        if was_connected:
            self._on_status("headset disconnected; arm command is held")

    def _on_receive_error(self, message: str) -> None:
        with self._lock:
            self._last_error = message
        self._on_status(message)

    def _on_decode_warning(self, message: str) -> None:
        with self._lock:
            self._decode_warning_count += 1
            self._last_error = message

    def _handle_packet(self, packet: Mapping[str, object], received_ns: int) -> None:
        with self._lock:
            self._packet_count += 1
        if packet.get("cmd") != CMD_FUNCTION:
            return
        tracking = TrackingDecoder.decode_tracking(packet)
        if tracking is None:
            return

        with self._lock:
            epoch = self._stream_epoch
        try:
            sample = parse_controller_sample(
                tracking,
                self.side,
                received_monotonic_ns=received_ns,
                stream_epoch=epoch,
            )
        except TrackingSampleError as exc:
            with self._lock:
                self._invalid_frame_count += 1
                self._last_error = str(exc)
            return

        clock_restarted = False
        with self._lock:
            timestamp_ns = sample.tracking_timestamp_ns
            if (
                timestamp_ns > 0
                and self._last_tracking_timestamp_ns > 0
                and timestamp_ns < self._last_tracking_timestamp_ns
            ):
                self._stream_epoch += 1
                epoch = self._stream_epoch
                clock_restarted = True
            if timestamp_ns > 0:
                self._last_tracking_timestamp_ns = timestamp_ns
            self._tracking_frame_count += 1

        if clock_restarted:
            self._buffer.clear()
            sample = replace(sample, stream_epoch=epoch)
            self._on_status("Tracking clock restarted; release Grip before rearming")
        self._buffer.publish(sample)
        if self._on_sample is not None:
            try:
                self._on_sample(sample, tracking)
            except Exception as exc:
                with self._lock:
                    self._last_error = f"sample callback failed: {exc}"


__all__ = [
    "CMD_FUNCTION",
    "MAX_BODY_SIZE",
    "PACKET_END",
    "PacketParser",
    "PacketStreamDecoder",
    "RECEIVE_PACKET_HEAD",
    "SEND_PACKET_HEAD",
    "TcpReceiver",
    "TrackingDecoder",
    "TrackingSourceStats",
    "V1TrackingSource",
]
