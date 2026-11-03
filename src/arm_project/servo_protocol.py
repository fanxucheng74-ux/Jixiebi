"""众灵总线舵机的纯协议函数：不导入硬件模块，不打开串口。

地址仅允许本项目的 000–005。P=500–2500 是协议值边界，
不是已标定的关节安全行程；实际运动的限位与许可由执行层另行检查。
位置回复不含时间戳或请求序号，解析成功不证明反馈新鲜或物理到位。
"""

# 左侧是程序使用的字段名，右侧是设备手册中的 ASCII 指令字节。
# b"PRAD" 是将写入串口的 bytes；这里不包含读写串口的动作。
READ_COMMANDS = {
    "position": b"PRAD",
    "mode": b"PMOD",
    "version": b"PVER",
    "id": b"PID",
    "telemetry": b"PRTV",
}


def _require_int(value, name, minimum, maximum):
    # bool 是 int 的子类，但不是本协议允许的数值参数。
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError("%s must be an integer in %d..%d" % (name, minimum, maximum))


def _require_bytes(value, name):
    # 串口层收发的是字节；拒绝 str，避免隐式编码改变实际协议内容。
    if type(value) is not bytes:
        raise ValueError(name + " must be bytes")


def _digits(value):
    # bytes 迭代得到整数；ASCII 数字 0～9 的字节值是 48～57。
    return bool(value) and all(48 <= byte <= 57 for byte in value)


def _require_field(field):
    if type(field) is not str or field not in READ_COMMANDS:
        raise ValueError("unsupported read field")


def make_query(servo_id, field="position"):
    """生成白名单只读查询之一，不允许广播、设置或动作。"""
    _require_int(servo_id, "servo_id", 0, 5)
    _require_field(field)
    # 例如 servo_id=4、field="position" 拼成 b"#004PRAD!"。
    # %03d 将地址补齐为三位十进制；# 是帧头，! 是帧尾。
    return ("#%03d" % servo_id).encode("ascii") + READ_COMMANDS[field] + b"!"


def make_position_query(servo_id):
    """返回单地址 PRAD 查询；不发送，也不允许广播。"""
    return make_query(servo_id, "position")


def validate_read_query(command):
    """只接受六个地址的精确只读查询，返回 (id, field)。"""
    _require_bytes(command, "command")
    # 按字节位置检查外壳：command[1:4] 取三位地址，不含下标 4。
    # PID 指令比 PRAD 等短一个字节，因此只读帧长度允许 8 或 9。
    if (
        len(command) not in (8, 9)
        or command[:1] != b"#"
        or command[-1:] != b"!"
        or not _digits(command[1:4])
    ):
        raise ValueError("not a supported read query")
    servo_id = int(command[1:4])
    _require_int(servo_id, "servo_id", 0, 5)
    # [4:-1] 是地址之后、末尾 ! 之前的指令字；只匹配白名单。
    for field, code in READ_COMMANDS.items():
        if command[4:-1] == code:
            return servo_id, field
    raise ValueError("not a supported read query")


def parse_position_reply(frame, expected_id):
    """解析 #dddPdddd! 并匹配地址；查询回显不是位置回复。"""
    _require_int(expected_id, "expected_id", 0, 5)
    _require_bytes(frame, "frame")
    # 位置帧 #004P1495!：地址在 [1:4]，字母 P 在 [4:5]，数值在 [5:9]。
    # 固定字段长度可避免把查询回显 #004PRAD! 当成数值回复。
    if (
        len(frame) != 10
        or frame[:1] != b"#"
        or frame[4:5] != b"P"
        or frame[-1:] != b"!"
        or not _digits(frame[1:4])
        or not _digits(frame[5:9])
    ):
        raise ValueError("not a position reply")
    servo_id = int(frame[1:4])
    _require_int(servo_id, "servo_id", 0, 5)
    # expected_id 是本次查询正在等待的舵机编号，防止误收其他轴的回复。
    if servo_id != expected_id:
        raise ValueError("reply servo_id does not match expected_id")
    position_p = int(frame[5:9])
    # 返回的是协议位置值 P；这里不把它换算成实际角度或空间坐标。
    _require_int(position_p, "position_p", 500, 2500)
    return position_p


def _telemetry_number(token, allow_negative=False):
    """有限长度十进制：1–3位整数，可带1位小数；温度允许负号。

    这个语法上限仅限制解析输入，不是温度量程或设备安全电压。
    不裁剪低电压/高电压，以免将需调查的实测值隐藏成正常值。
    """
    original = token
    if allow_negative and token[:1] == b"-":
        token = token[1:]
    parts = token.split(b".")
    # 拆开整数和小数部分，先验证有限格式，再转换为 Python 浮点数。
    if len(parts) not in (1, 2) or not 1 <= len(parts[0]) <= 3 or not _digits(parts[0]):
        raise ValueError("invalid telemetry decimal")
    if len(parts) == 2 and (len(parts[1]) != 1 or not _digits(parts[1])):
        raise ValueError("invalid telemetry decimal")
    return float(original.decode("ascii"))


def parse_reply(frame, expected_id, field):
    """兼容已确认回包及手册遥测示例；只接收指定地址和类型。

    手册telemetry返回temperature_c/voltage_v，不表示供电或温度安全。
    实物@TdddVddd仅保留temperature_raw/voltage_raw与units_verified=False；
    该固件缩放和单位尚未核实，不将三位原始字段猜测为物理量。
    """
    _require_int(expected_id, "expected_id", 0, 5)
    _require_field(field)
    _require_bytes(frame, "frame")
    if field == "position":
        return parse_position_reply(frame, expected_id)
    if len(frame) < 6 or frame[:1] != b"#" or frame[-1:] != b"!" or not _digits(frame[1:4]):
        raise ValueError("not a supported reply")
    servo_id = int(frame[1:4])
    _require_int(servo_id, "servo_id", 0, 5)
    if servo_id != expected_id:
        raise ValueError("reply servo_id does not match expected_id")
    # 去掉 #、三位地址和结尾 ! 后，只对载荷 body 判断回复类型。
    body = frame[4:-1]
    # PVER 以 PV 开头；必须先排除查询回显，不能将其误读成版本 ER。
    if body in READ_COMMANDS.values():
        raise ValueError("query echo is not a reply")
    if field == "id" and body == b"P":
        return servo_id
    if field == "telemetry" and body[:2] == b"@T":
        # 2026-10-31 实物RX：#000@T090V082!。保留前导零，不猜小数点。
        if len(body) == 9 and body[5:6] == b"V" and _digits(body[2:5]) and _digits(body[6:9]):
            return {
                "temperature_raw": body[2:5].decode("ascii"),
                "voltage_raw": body[6:9].decode("ascii"),
                "units_verified": False,
            }
        raise ValueError("invalid raw telemetry reply")
    if field == "telemetry" and body[:1] == b"T":
        # 手册示例形如 T25.0V8.4；V 将温度字段与电压字段分开。
        parts = body[1:].split(b"V")
        if len(parts) == 2:
            return {
                "temperature_c": _telemetry_number(parts[0], allow_negative=True),
                "voltage_v": _telemetry_number(parts[1]),
            }
    if field == "mode":
        # 兼容已确认的 MODx / PMODx 两种写法；x 仍限于枚举 1～8。
        value = None
        if len(body) == 4 and body[:3] == b"MOD":
            value = body[3:]
        elif len(body) == 5 and body[:4] == b"PMOD":
            value = body[4:]
        if value is not None and value in (b"1", b"2", b"3", b"4", b"5", b"6", b"7", b"8"):
            return int(value)
    if field == "version":
        # 版本是受限文本而非数值；保留手册/实测的前缀差异，不执行它。
        allowed = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_.-"
        version = None
        if body[:2] == b"@ ":
            version = body[2:]
            allowed += b" "
        elif body[:2] == b"PV":
            version = body[2:]
        if version and version.strip() and all(byte in allowed for byte in version):
            return version.decode("ascii")
    raise ValueError("reply type or format does not match requested field")


def make_move_command(servo_id, position_p, duration_ms):
    """只生成定位帧；通过协议边界检查不代表允许执行实物动作。"""
    _require_int(servo_id, "servo_id", 0, 5)
    # position_p 是目标协议值；duration_ms 是指令声明的运动时长（毫秒）。
    # 这些是字段合法范围，关节限位、路径、预算仍由上层控制器检查。
    _require_int(position_p, "position_p", 500, 2500)
    _require_int(duration_ms, "duration_ms", 20, 9999)
    # 例如 (4, 1540, 1500) → #004P1540T1500!；%04d 将 P/T 各补齐四位。
    return ("#%03dP%04dT%04d!" % (servo_id, position_p, duration_ms)).encode("ascii")


def validate_motion_command(command):
    """精确验证单地址定位帧，返回 (id, P, T)，不接受其他命令。"""
    _require_bytes(command, "command")
    # 定位帧共 15 字节：# + 三位 ID + P + 四位位置 + T + 四位时长 + !。
    # 固定分隔字母与数字切片必须同时正确，不能只检查命令是否以 # 开头。
    if (
        len(command) != 15
        or command[:1] != b"#"
        or command[4:5] != b"P"
        or command[9:10] != b"T"
        or command[-1:] != b"!"
        or not _digits(command[1:4])
        or not _digits(command[5:9])
        or not _digits(command[10:14])
    ):
        raise ValueError("not a supported motion command")
    servo_id = int(command[1:4])
    position_p = int(command[5:9])
    duration_ms = int(command[10:14])
    # 保持组帧与验证使用同一套字段边界。
    make_move_command(servo_id, position_p, duration_ms)
    return servo_id, position_p, duration_ms


class FrameReceiver:
    """跨多次 feed 拼出 #...!，仅分帧，不判断业务类型或新鲜度。

    max_frame_bytes 包含 # 与 !；默认 64 是接收策略，不是厂家上限。
    新 # 重新同步；超长片段丢弃；! 之后无新 # 的字节视为噪声。
    """

    def __init__(self, max_frame_bytes=64):
        if type(max_frame_bytes) is not int or max_frame_bytes < 2:
            raise ValueError("max_frame_bytes must be an integer >= 2")
        self.max_frame_bytes = max_frame_bytes
        # buffer 属于接收器实例；一次 feed 未收齐的半帧留给下一次 feed。
        self.buffer = bytearray()

    def feed(self, chunk):
        if not isinstance(chunk, (bytes, bytearray, memoryview)):
            raise ValueError("chunk must be bytes, bytearray or memoryview")
        frames = []
        for byte in chunk:
            if byte == 35:  # #：也能从未结束的坏帧重新同步。
                self.buffer = bytearray(b"#")
            elif self.buffer:
                self.buffer.append(byte)
                if len(self.buffer) > self.max_frame_bytes:
                    # 丢弃这一段坏帧，等待新的 #；不把超长内容继续累计到内存。
                    self.buffer = bytearray()
                elif byte == 33:  # !
                    # 完整帧复制成不可变 bytes；清空工作缓冲后还能提取后续粘连帧。
                    frames.append(bytes(self.buffer))
                    self.buffer = bytearray()
        return frames
