"""Persistent TCP client for the YOLO11 server running inside WSL."""

from __future__ import annotations

import socket
import struct

from core.yolo11_detector import Box


_HEADER = struct.Struct("!4sHHI")
_REQUEST = struct.Struct("!QIII")
_RESPONSE = struct.Struct("!QHHQ")
_COUNT = struct.Struct("!I")
_BOX = struct.Struct("!iiiiif")
_MAGIC = b"Y11T"
_VERSION = 1
_REQUEST_DETECT = 1
_RESPONSE_DETECT = 0x8001
_MAX_BOXES = 1000
_MAX_RESPONSE = _RESPONSE.size + _COUNT.size + _MAX_BOXES * _BOX.size


def _recv_exact(connection: socket.socket, size: int) -> bytes:
    data = bytearray()
    while len(data) < size:
        chunk = connection.recv(size - len(data))
        if not chunk:
            raise ConnectionError("WSL YOLO 서버 연결이 끊겼습니다")
        data.extend(chunk)
    return bytes(data)


class TcpDetector:
    """Return the same Box values as the local DLL detector."""

    def __init__(self, host: str, port: int, timeout_s: float = 3.0) -> None:
        self.host = host
        self.port = port
        self._connection = socket.create_connection((host, port), timeout=timeout_s)
        self._connection.settimeout(timeout_s)
        self._connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def detect(
        self, bgra: bytes, width: int, height: int, stride_bytes: int = 0,
        frame_id: int | None = None,
    ) -> list[Box]:
        if frame_id is None or not 0 <= frame_id <= 0xFFFFFFFFFFFFFFFF:
            raise ValueError("CARLA frame number is required for WSL detection")
        stride = stride_bytes or width * 4
        if (not 1 <= width <= 3000 or not 1 <= height <= 3000
                or stride < width * 4 or stride > width * 4 + 4096
                or len(bgra) != stride * height):
            raise ValueError("Invalid BGRA frame")
        payload_size = _REQUEST.size + len(bgra)
        if payload_size > 64 * 1024 * 1024:
            raise ValueError("BGRA frame exceeds YOLO server limit")

        self._connection.sendall(
            _HEADER.pack(_MAGIC, _VERSION, _REQUEST_DETECT, payload_size)
            + _REQUEST.pack(frame_id, width, height, stride)
            + bgra
        )
        magic, version, message_type, size = _HEADER.unpack(
            _recv_exact(self._connection, _HEADER.size)
        )
        if (magic, version, message_type) != (_MAGIC, _VERSION, _RESPONSE_DETECT):
            raise RuntimeError("WSL YOLO 서버 응답 프로토콜이 일치하지 않습니다")
        if size < _RESPONSE.size or size > _MAX_RESPONSE:
            raise RuntimeError("WSL YOLO 서버 응답 크기가 유효하지 않습니다")
        response = _recv_exact(self._connection, size)
        response_frame, status, _reserved, _inference_us = _RESPONSE.unpack_from(response)
        if response_frame != frame_id:
            raise RuntimeError(
                f"WSL YOLO 프레임 불일치: 요청 {frame_id}, 응답 {response_frame}"
            )
        if status:
            message = response[_RESPONSE.size:].decode("utf-8", errors="replace")
            raise RuntimeError(f"WSL YOLO 서버 오류 {status}: {message}")
        if size < _RESPONSE.size + _COUNT.size:
            raise RuntimeError("WSL YOLO 서버 응답에 검출 개수가 없습니다")
        (count,) = _COUNT.unpack_from(response, _RESPONSE.size)
        if count > _MAX_BOXES or size != _RESPONSE.size + _COUNT.size + count * _BOX.size:
            raise RuntimeError("WSL YOLO 검출 박스 데이터 길이가 유효하지 않습니다")
        return [Box(*_BOX.unpack_from(response, _RESPONSE.size + _COUNT.size + i * _BOX.size))
                for i in range(count)]

    def close(self) -> None:
        self._connection.close()
