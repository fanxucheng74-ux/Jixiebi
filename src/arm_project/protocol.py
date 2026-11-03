"""电脑第一课：有长度边界的 JSON 行协议，不含硬件操作。"""

import json

MAX_FRAME_BYTES = 1024  # 一条 JSON 的 UTF-8 字节数，不含结尾换行。
PROTOCOL_VERSION = 1  # 通信双方约定的数据格式版本，当前只接收版本 1。


class ProtocolError(ValueError):
    pass


def validate(message):
    # message 是解码后的字典；先检查公共外壳，具体业务载荷由使用者继续验证。
    if not isinstance(message, dict):
        raise ProtocolError("message must be a JSON object")
    version = message.get("protocol_version")
    if type(version) is not int or version != PROTOCOL_VERSION:
        raise ProtocolError("unsupported protocol_version")
    # type 表示消息用途，command_id 用于把应答与原请求关联；限制长度避免无界输入。
    for key, limit in (("type", 32), ("command_id", 64)):
        value = message.get(key)
        if not isinstance(value, str) or not 1 <= len(value) <= limit:
            raise ProtocolError("invalid " + key)
    return message


def encode(message):
    validate(message)
    try:
        # 先序列化再编码 UTF-8：中文通常占多个字节，因此上限必须按字节计。
        # allow_nan=False 排除 NaN/Infinity；紧凑分隔符去掉不必要的空格。
        frame = json.dumps(
            message, ensure_ascii=False, allow_nan=False, separators=(",", ":")
        ).encode("utf-8")
    except (ValueError, TypeError, UnicodeError) as error:
        raise ProtocolError("message is not valid JSON") from error
    if len(frame) > MAX_FRAME_BYTES:
        raise ProtocolError("frame exceeds 1024 bytes")
    # TCP 本身没有消息边界，约定每个 JSON 末尾追加换行作为分帧标志。
    return frame + b"\n"


def _reject_constant(value):
    # 接收端也拒绝非标准 JSON 常量，与发送端的 allow_nan=False 保持一致。
    raise ProtocolError("invalid JSON constant: " + value)


class MessageStream:
    """缓存收到的字节，直到找到换行；保留后面的下一帧。"""

    def __init__(self, connection):
        # connection 持有已建立的 socket；本类不负责建连、设置超时或关闭它。
        self.connection = connection
        # 持久缓冲区保存半帧，也保存一次 recv 中多收到的下一帧。
        self.buffer = bytearray()

    def send(self, message):
        # sendall 发送整段编码字节，失败会抛异常；发送成功不代表对端业务已完成。
        self.connection.sendall(encode(message))

    def receive(self):
        while True:
            # 先消费已有缓冲，避免已有完整消息时仍阻塞等待新的网络数据。
            end = self.buffer.find(b"\n")
            if end >= 0:
                if end > MAX_FRAME_BYTES:
                    raise ProtocolError("frame exceeds 1024 bytes")
                frame = bytes(self.buffer[:end])
                # 只移除当前帧和它的换行；下一条消息可能已在 buffer 后半段。
                del self.buffer[: end + 1]
                try:
                    # 完整字节帧先按 UTF-8 解码成文本，再解析成字典并检查公共字段。
                    message = json.loads(frame.decode("utf-8"), parse_constant=_reject_constant)
                except (ValueError, UnicodeError, RecursionError) as error:
                    raise ProtocolError("invalid JSON frame") from error
                return validate(message)
            if len(self.buffer) > MAX_FRAME_BYTES:
                # 还没找到换行就超长，同样拒绝，防止发送方无限发送未结束帧。
                raise ProtocolError("frame exceeds 1024 bytes")
            # 512 是每次读取的最大字节数，并不表示一次一定收到 512 字节或完整消息。
            chunk = self.connection.recv(512)
            if not chunk:
                # 空 bytes 表示连接结束；缓冲非空说明最后一帧被截断。
                if self.buffer:
                    raise ProtocolError("connection ended during a frame")
                raise EOFError("peer closed the connection")
            self.buffer.extend(chunk)


def validate_reply(reply, request, expected_type):
    validate(reply)
    # 这是网络 JSON 层的关联校验；不能补出底层舵机协议里不存在的请求序号。
    if reply["command_id"] != request["command_id"]:
        raise ProtocolError("response command_id does not match request")
    if reply["type"] != expected_type:
        # expected_type 由调用方给出，防止同一 command_id 的其他类型消息被误用。
        raise ProtocolError("unexpected response type: " + reply["type"])
