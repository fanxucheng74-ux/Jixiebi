"""电脑模拟 ESP32，仅提供查询与回声，不控制舵机。"""

import argparse
import socket
import uuid

from arm_project.protocol import MessageStream, ProtocolError, PROTOCOL_VERSION

COMMANDS = ("HELLO", "PING", "GET_STATUS")


def make_reply(request, boot_id):
    reply = {"protocol_version": PROTOCOL_VERSION, "command_id": request["command_id"]}
    kind = request["type"]
    if kind == "HELLO":
        reply.update(
            type="HELLO_ACK",
            payload={
                "device": "PC_ESP32_SIMULATOR",
                "boot_id": boot_id,
                "commands": list(COMMANDS),
                "hardware_connected": False,
            },
        )
    elif kind == "PING":
        # 第一课原样返回消息，让学生观察请求和响应。
        reply.update(type="PONG", payload=request.get("payload", {}))
    elif kind == "GET_STATUS":
        reply.update(
            type="STATUS",
            payload={
                "mode": "SIMULATION",
                "state": "IDLE",
                "hardware_connected": False,
                "motion_enabled": False,
                "wifi_verified": False,
            },
        )
    else:
        reply.update(
            type="ERROR",
            payload={
                "code": "UNSUPPORTED_COMMAND",
                "detail": "This lesson supports HELLO, PING, GET_STATUS only.",
            },
        )
    return reply


def serve_connection(connection, boot_id, timeout=30):
    connection.settimeout(timeout)
    stream = MessageStream(connection)
    try:
        while True:
            request = stream.receive()
            print("RX", request["type"], request["command_id"], flush=True)
            stream.send(make_reply(request, boot_id))
    except EOFError:
        print("Client disconnected; waiting for next connection.", flush=True)
    except (ProtocolError, OSError) as error:
        # 畸形数据或超时只结束当前连接，外层服务继续运行。
        print("Connection closed:", error, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    if not 0 <= args.port <= 65535:
        parser.error("port must be between 0 and 65535")
    boot_id = uuid.uuid4().hex
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
            # 第一课固定本机地址，不需要开放防火墙或接入无线网络。
            listener.bind(("127.0.0.1", args.port))
            listener.listen(1)
            listener.settimeout(0.5)
            port = listener.getsockname()[1]
            print(f"SIMULATOR listening on 127.0.0.1:{port}", flush=True)
            print("No hardware connected. Stop with Ctrl+C.", flush=True)
            while True:
                try:
                    connection, _ = listener.accept()
                except socket.timeout:
                    continue
                with connection:
                    serve_connection(connection, boot_id)
    except KeyboardInterrupt:
        print("\nSimulator stopped.")
    except OSError as error:
        print("Cannot start simulator:", error)
        print("If the port is occupied, stop the old simulator or use --port 8766.")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
