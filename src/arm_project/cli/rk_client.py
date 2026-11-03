"""电脑模拟 RK 上位机：连接、问候、回声、查询状态。"""

import argparse
import json
import socket
import uuid

from arm_project.protocol import MessageStream, ProtocolError, PROTOCOL_VERSION, validate_reply


def run(host, port, message):
    with socket.create_connection((host, port), timeout=3) as connection:
        connection.settimeout(3)
        stream = MessageStream(connection)
        actions = [
            ("HELLO", {}, "HELLO_ACK"),
            ("PING", {"text": message}, "PONG"),
            ("GET_STATUS", {}, "STATUS"),
        ]
        for kind, payload, expected in actions:
            request = {
                "protocol_version": PROTOCOL_VERSION,
                "command_id": uuid.uuid4().hex,
                "type": kind,
                "payload": payload,
            }
            print("SEND", kind)
            stream.send(request)
            reply = stream.receive()
            validate_reply(reply, request, expected)
            if kind == "PING" and reply.get("payload") != payload:
                raise ProtocolError("echo content does not match request")
            print("RECV", reply["type"], json.dumps(reply.get("payload", {}), ensure_ascii=False))
    print("PASS: 3 TCP exchanges completed; PC simulation only.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--message", default="hello from RK")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("port must be between 1 and 65535")
    try:
        run(args.host, args.port, args.message)
    except (OSError, EOFError, ProtocolError) as error:
        print("Communication failed:", error)
        print("Start sim_esp32.py first; check that both programs use the same port.")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
