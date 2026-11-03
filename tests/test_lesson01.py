import contextlib
import io
import socket
import threading
import unittest

from arm_project.protocol import MessageStream, ProtocolError, encode, validate_reply
from arm_project.cli.sim_esp32 import serve_connection


def request(kind="PING", command_id="test-1", payload=None):
    return {"protocol_version": 1, "type": kind, "command_id": command_id, "payload": payload or {}}


class Chunks:
    """确定性模拟 TCP 任意分段，避免把某次系统分包当协议保证。"""

    def __init__(self, chunks):
        self.chunks = iter(chunks)

    def recv(self, size):
        return next(self.chunks, b"")


class ProtocolTests(unittest.TestCase):
    def test_utf8_split_at_every_byte(self):
        original = request(payload={"text": "红绿蓝木块"})
        stream = MessageStream(Chunks([bytes([b]) for b in encode(original)]))
        self.assertEqual(stream.receive(), original)

    def test_multiple_frames_keep_remainder(self):
        originals = [request(command_id=str(n)) for n in range(20)]
        stream = MessageStream(Chunks([b"".join(map(encode, originals))]))
        self.assertEqual([stream.receive() for _ in originals], originals)

    def test_exact_limit_and_overlong_with_or_without_newline(self):
        prefix = b'{"protocol_version":1,"type":"PING","command_id":"x","payload":"'
        exact = prefix + b"a" * (1024 - len(prefix) - 2) + b'"}'
        self.assertEqual(len(exact), 1024)
        self.assertEqual(MessageStream(Chunks([exact, b"\n"])).receive()["type"], "PING")
        for frame in (exact + b" ", exact + b" \n"):
            with self.subTest(frame_length=len(frame)):
                with self.assertRaises(ProtocolError):
                    MessageStream(Chunks([frame])).receive()

    def test_invalid_inputs_and_partial_disconnect(self):
        frames = [
            b"oops\n",
            b"[]\n",
            b"\xff\n",
            b"\n",
            b'{"protocol_version":true,"type":"PING","command_id":"x"}\n',
            b'{"protocol_version":1,"type":"PING","command_id":"x","n":NaN}\n',
            b'{"protocol_version":',
        ]
        for frame in frames:
            with self.subTest(frame=frame), self.assertRaises(ProtocolError):
                MessageStream(Chunks([frame])).receive()
        with self.assertRaises(EOFError):
            MessageStream(Chunks([])).receive()

    def test_reply_must_match_id_and_type(self):
        original = request()
        for response in (request("PONG", "wrong-id"), request("STATUS")):
            with self.assertRaises(ProtocolError):
                validate_reply(response, original, "PONG")


class LoopbackTests(unittest.TestCase):
    def test_real_tcp_recovery_and_simulation_identity(self):
        # 使用操作系统分配的空闲端口；只监听回环地址。
        errors = []
        with socket.socket() as listener:
            listener.bind(("127.0.0.1", 0))
            listener.listen(1)
            listener.settimeout(3)
            address = listener.getsockname()

            def worker():
                try:
                    for _ in range(3):
                        connection, _ = listener.accept()
                        with connection:
                            serve_connection(connection, "test-boot", timeout=1)
                except Exception as error:
                    errors.append(error)

            with contextlib.redirect_stdout(io.StringIO()):
                thread = threading.Thread(target=worker, daemon=True)
                thread.start()
                with socket.create_connection(address, timeout=3) as client:
                    stream = MessageStream(client)
                    messages = [
                        request("HELLO", "1"),
                        request("PING", "2", {"text": "红"}),
                        request("GET_STATUS", "3"),
                        request("MOVE", "4"),
                    ]
                    # 一次发送多条请求，检查服务不会吞掉后续消息。
                    client.sendall(b"".join(map(encode, messages)))
                    replies = [stream.receive() for _ in messages]
                    self.assertEqual([r["command_id"] for r in replies], ["1", "2", "3", "4"])
                    self.assertEqual(
                        [r["type"] for r in replies], ["HELLO_ACK", "PONG", "STATUS", "ERROR"]
                    )
                    self.assertEqual(replies[1]["payload"], {"text": "红"})
                    self.assertFalse(replies[2]["payload"]["motion_enabled"])
                    self.assertFalse(replies[2]["payload"]["wifi_verified"])
                with socket.create_connection(address, timeout=3) as client:
                    client.sendall(b"invalid JSON\n")
                    self.assertEqual(client.recv(1), b"")
                # 畸形连接结束之后，新的合法连接仍能完成查询。
                with socket.create_connection(address, timeout=3) as client:
                    stream = MessageStream(client)
                    stream.send(request("GET_STATUS"))
                    self.assertEqual(stream.receive()["type"], "STATUS")
                thread.join(timeout=5)
                self.assertFalse(thread.is_alive())
                self.assertEqual(errors, [])


if __name__ == "__main__":
    unittest.main()
