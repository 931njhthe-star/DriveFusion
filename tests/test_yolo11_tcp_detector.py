"""Protocol tests for the persistent WSL YOLO client."""

import socket
import struct
import threading
import unittest

from core.yolo11_tcp_detector import TcpDetector


HEADER = struct.Struct("!4sHHI")
REQUEST = struct.Struct("!QIII")
RESPONSE = struct.Struct("!QHHQ")
COUNT = struct.Struct("!I")
BOX = struct.Struct("!iiiiif")


def recv_exact(connection, length):
    output = bytearray()
    while len(output) < length:
        chunk = connection.recv(length - len(output))
        if not chunk:
            raise ConnectionError("client disconnected")
        output.extend(chunk)
    return bytes(output)


class TcpDetectorTests(unittest.TestCase):
    def _serve(self, count=2, wrong_frame=False):
        listener = socket.socket()
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        port = listener.getsockname()[1]
        seen = []

        def serve():
            try:
                connection, _ = listener.accept()
                with connection:
                    for _ in range(count):
                        magic, version, kind, size = HEADER.unpack(recv_exact(connection, 12))
                        payload = recv_exact(connection, size)
                        frame_id, width, height, stride = REQUEST.unpack_from(payload)
                        seen.append((magic, version, kind, frame_id, width, height, stride, payload[20:]))
                        result = (RESPONSE.pack(frame_id + int(wrong_frame), 0, 0, 1234)
                                  + COUNT.pack(1) + BOX.pack(3, 4, 5, 6, 2, 0.75))
                        packet = HEADER.pack(b"Y11T", 1, 0x8001, len(result)) + result
                        for offset in range(0, len(packet), 5):
                            connection.sendall(packet[offset:offset + 5])
            finally:
                listener.close()

        thread = threading.Thread(target=serve, daemon=True)
        thread.start()
        return port, seen, thread

    def test_persistent_connection_and_fragmented_response(self):
        port, seen, thread = self._serve()
        detector = TcpDetector("127.0.0.1", port)
        image = bytes(range(32))
        try:
            for frame_id in (41, 42):
                boxes = detector.detect(image, 2, 4, frame_id=frame_id)
                self.assertEqual((boxes[0].x, boxes[0].class_id, boxes[0].confidence),
                                 (3, 2, 0.75))
        finally:
            detector.close()
            thread.join(timeout=2)
        self.assertEqual([item[3] for item in seen], [41, 42])
        self.assertTrue(all(item[:3] == (b"Y11T", 1, 1) for item in seen))
        self.assertTrue(all(item[4:] == (2, 4, 8, image) for item in seen))

    def test_rejects_response_from_different_frame(self):
        port, _, thread = self._serve(count=1, wrong_frame=True)
        detector = TcpDetector("127.0.0.1", port)
        try:
            with self.assertRaisesRegex(RuntimeError, "프레임 불일치"):
                detector.detect(bytes(8), 1, 2, frame_id=9)
        finally:
            detector.close()
            thread.join(timeout=2)


if __name__ == "__main__":
    unittest.main()
