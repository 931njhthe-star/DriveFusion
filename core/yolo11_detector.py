"""ctypes bridge for the cross-platform YOLO11 TensorRT C++ library."""

from __future__ import annotations

import ctypes as C
from dataclasses import dataclass
import os
from pathlib import Path


class _CBox(C.Structure):
    _fields_ = [
        ("x", C.c_int),
        ("y", C.c_int),
        ("width", C.c_int),
        ("height", C.c_int),
        ("class_id", C.c_int),
        ("confidence", C.c_float),
    ]


@dataclass(frozen=True)
class Box:
    x: int
    y: int
    width: int
    height: int
    class_id: int
    confidence: float


class Detector:
    """One instance per process; call detect from a worker, not the Qt UI thread."""

    def __init__(
        self,
        library_path: str | Path,
        engine_path: str | Path,
        gpu_id: int = 0,
        dll_directories: tuple[str | Path, ...] = (),
    ) -> None:
        self._dll_dirs = []
        if os.name == "nt":
            for directory in dll_directories:
                self._dll_dirs.append(os.add_dll_directory(str(directory)))
        self._lib = C.CDLL(str(library_path))
        self._lib.yolo11_create.argtypes = [C.c_char_p, C.c_int, C.POINTER(C.c_char), C.c_size_t]
        self._lib.yolo11_create.restype = C.c_void_p
        self._lib.yolo11_destroy.argtypes = [C.c_void_p]
        self._lib.yolo11_destroy.restype = None
        self._lib.yolo11_detect_bgra.argtypes = [
            C.c_void_p, C.c_char_p, C.c_int, C.c_int, C.c_size_t,
            C.POINTER(_CBox), C.c_size_t, C.POINTER(C.c_char), C.c_size_t,
        ]
        self._lib.yolo11_detect_bgra.restype = C.c_int
        error = C.create_string_buffer(1024)
        self._handle = self._lib.yolo11_create(
            os.fsencode(engine_path), gpu_id, error, len(error)
        )
        if not self._handle:
            raise RuntimeError(error.value.decode("utf-8", errors="replace"))

    def detect(self, bgra: bytes, width: int, height: int, stride_bytes: int = 0) -> list[Box]:
        if not self._handle:
            raise RuntimeError("Detector is closed")
        stride = stride_bytes or width * 4
        if width <= 0 or height <= 0 or stride < width * 4 or len(bgra) < stride * height:
            raise ValueError("Invalid BGRA frame")
        boxes = (_CBox * 1000)()
        error = C.create_string_buffer(1024)
        count = self._lib.yolo11_detect_bgra(
            self._handle, bgra, width, height, stride,
            boxes, len(boxes), error, len(error),
        )
        if count < 0:
            raise RuntimeError(error.value.decode("utf-8", errors="replace"))
        return [Box(b.x, b.y, b.width, b.height, b.class_id, b.confidence)
                for b in boxes[:count]]

    def close(self) -> None:
        if self._handle:
            self._lib.yolo11_destroy(self._handle)
            self._handle = None
        for directory in self._dll_dirs:
            directory.close()
        self._dll_dirs.clear()

    def __enter__(self) -> Detector:
        return self

    def __exit__(self, _exception_type: object, _exception: object, _traceback: object) -> None:
        self.close()
