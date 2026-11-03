"""只在显式打开时连接ZLink；协议与串口、回放分离。"""

from datetime import datetime
import sys
import time

from arm_project.servo_protocol import FrameReceiver, make_query, parse_reply
from arm_project.servo_protocol import validate_read_query, validate_motion_command


class Clock:
    # monotonic只用于计算经过时间/截止时间，不受系统校时影响，单位为秒。
    # timestamp用于人读日志；两种时间不能混用来判断串口超时。
    def monotonic(self):
        return time.monotonic()

    def sleep(self, seconds):
        time.sleep(seconds)

    def timestamp(self):
        return datetime.now().astimezone().isoformat()


class SerialTransport:
    source = "HARDWARE"

    def __init__(self, port_name):
        self.port_name = port_name
        self.port = None

    def open(self):
        # 延迟导入串口库：导入本文件、构造对象和跑离线测试都不会连接设备。
        import serial

        # 115200/8N1为本项目已确认参数；read每次最多等20ms，事务总超时另由ServoLink管理。
        port = serial.Serial(
            port=None,
            baudrate=115200,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=0.02,
            write_timeout=0.3,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        # 先配置DTR/RTS再显式open，避免依赖库默认控制线状态；不把它当成设备复位流程。
        port.dtr = False
        port.rts = False
        port.port = self.port_name
        try:
            if sys.platform.startswith("linux"):
                # pyserial 3.5在open时申请非阻塞flock建议锁；冲突即失败，不自动重试。
                # 只约束同样遵守锁的客户端，不能阻止不检查建议锁的其他程序打开端口。
                port.exclusive = True
            port.open()
        except BaseException:
            port.close()
            raise
        self.port = port

    @property
    def in_waiting(self):
        return self.port.in_waiting

    def read(self, count):
        return self.port.read(count)

    def write(self, command):
        # 第二道限制：禁止广播、改ID、模式设置、释力和恢复出厂。
        # 这是早期底座局部验收的传输白名单；统一驱动使用自己的精确计划网关。
        if command != b"#000PDST!":
            try:
                validate_read_query(command)
            except ValueError:
                servo_id, position, duration = validate_motion_command(command)
                if servo_id != 0 or not 1505 <= position <= 1555 or not 1500 <= duration <= 5000:
                    raise ValueError("hardware motion outside reviewed base-only envelope")
        return self.port.write(command)

    def close(self):
        if self.port is not None:
            self.port.close()

    @property
    def closed(self):
        return self.port is None or not self.port.is_open


class ReplayTransport:
    """按原日志查询顺序返回原始RX字节，不产生真实设备状态。"""

    source = "REPLAY"

    def __init__(self, exchanges):
        # exchanges是历史“发送查询→接收字节”的顺序表；pending模拟尚未读出的串口字节。
        self.exchanges = list(exchanges)
        self.index = 0
        self.pending = bytearray()
        self.writes = []
        self.closed = True

    def open(self):
        self.closed = False

    @property
    def in_waiting(self):
        return len(self.pending)

    def read(self, count):
        # 只移走本次读到的前count字节，剩余数据保留给下一次read，模拟真实流式读取。
        result = bytes(self.pending[:count])
        del self.pending[:count]
        return result

    def write(self, command):
        # 回放只接受下一条已记录的查询；不合成运动结果或新的物理设备状态。
        validate_read_query(command)
        if self.index >= len(self.exchanges):
            raise OSError("replay has no more recorded queries")
        exchange = self.exchanges[self.index]
        if command.decode("ascii") != exchange["command"]:
            raise OSError("query does not match the next recorded exchange")
        self.index += 1
        self.writes.append(command)
        # rx_hex是原始十六进制字节记录，先解码再放入接收缓冲，解析仍交给上层协议代码。
        self.pending.extend(b"".join(bytes.fromhex(h) for h in exchange["rx_hex"]))
        return len(command)

    def close(self):
        self.pending.clear()
        self.closed = True


class ServoLink:
    """一问一答。只供一个控制线程使用；故障后丢弃本次会话。"""

    def __init__(self, transport, record=None, clock=None, timeout=0.5):
        if not 0.05 <= timeout <= 2:
            raise ValueError("timeout must be between 0.05 and 2 seconds")
        # transport负责字节收发，clock负责时间，record负责审计；三者都可注入离线替身。
        self.transport = transport
        self.clock = clock or Clock()
        self.record = record or (lambda event, **values: None)
        self.timeout = timeout
        self.faulted = False
        self.log_errors = []

    @property
    def source(self):
        return self.transport.source

    def __enter__(self):
        # with ServoLink(...)进入时才打开设备；进入阶段出错也必须释放串口句柄。
        self.transport.open()
        try:
            self.record(
                "opened", source=self.source, port=getattr(self.transport, "port_name", None)
            )
            self.clock.sleep(0.1)
        except BaseException:
            try:
                self.transport.close()
            except BaseException as error:
                self.log_errors.append("close during enter failure: " + str(error))
            raise
        return self

    def __exit__(self, *args):
        # 退出with优先关串口；已有异常时，清理失败仅记日志，不掩盖最初故障。
        try:
            self.transport.close()
        except BaseException as error:
            self.record_best_effort("close_failed", error=str(error))
            if args[0] is None:
                raise
        else:
            self.record_best_effort("closed")
        finally:
            self.record_best_effort("session_exit")

    def record_best_effort(self, event, **values):
        # 日志故障不能阻止异常清理或覆盖原始通信/运动故障。
        try:
            self.record(event, **values)
        except Exception as error:
            self.log_errors.append("%s: %s" % (event, error))

    def query(self, servo_id, field="position"):
        # make_query只组帧；本方法才执行“写一次请求→等待指定地址和字段的回复”。
        command = make_query(servo_id, field)
        if self.faulted:
            raise RuntimeError("link is faulted; end session before reconnecting")
        try:
            # 记录并排除请求前已在缓冲中的字节；协议无序号，不能消除所有旧回复歧义。
            if self.transport.in_waiting:
                pending = self.transport.read(min(self.transport.in_waiting, 4096))
                self.record("pending_discarded", rx_hex=pending.hex())
                if self.transport.in_waiting:
                    raise OSError("too much pending input")
            receiver = FrameReceiver()
            started = self.clock.monotonic()
            # deadline是主机单调时钟上的绝对截止点，不能在每次read后重新延长。
            deadline = started + self.timeout
            self.record("tx", command=command.decode("ascii"))
            if self.transport.write(command) != len(command):
                raise OSError("query short write; not retried")
            total = 0
            while self.clock.monotonic() < deadline:
                # in_waiting为当前已缓冲字节数；没有现成字节时读1字节，仍受串口短超时限制。
                # 一次read可能是半帧或多帧，不能直接当作一条位置回复。
                chunk = self.transport.read(min(self.transport.in_waiting, 256) or 1)
                received = self.clock.monotonic()
                if not chunk:
                    self.clock.sleep(0.001)
                    continue
                self.record("rx", rx_hex=chunk.hex())
                total += len(chunk)
                # 4096是本事务接收预算，防止杂波持续输入使事务无限消耗资源。
                if total > 4096:
                    raise OSError("receive byte budget exceeded")
                if received >= deadline:
                    break
                matched = None
                for frame in receiver.feed(chunk):
                    try:
                        # 同时匹配servo_id和field，回显、其他舵机回复或格式错误都会被排除。
                        value = parse_reply(frame, servo_id, field)
                    except ValueError as error:
                        self.record("rejected", frame_hex=frame.hex(), reason=str(error))
                        continue
                    if matched is None:
                        # received_*表示主机收到这批字节的时间，不是舵机采样时间。
                        matched = {
                            "value": value,
                            "received_at": self.clock.timestamp(),
                            "received_monotonic_s": received,
                        }
                    else:
                        self.record("extra_reply_discarded", frame_hex=frame.hex())
                if matched is not None:
                    # 一问一答只返回首个匹配值；本事务多余帧和残片写入日志后丢弃。
                    if receiver.buffer:
                        self.record("tail_discarded", rx_hex=bytes(receiver.buffer).hex())
                    self.record("reply", servo_id=servo_id, field=field, **matched)
                    return matched
            raise TimeoutError("no matching reply within query deadline")
        except (OSError, RuntimeError, ValueError):
            # 事务故障后锁住本连接；调用者必须结束会话，不自动补发或继续下一条运动。
            self.faulted = True
            raise

    def send_motion(self, command):
        # 这里只保证帧合法且完整写出一次；有没有运动、是否到位由驱动读取反馈另行判断。
        validate_motion_command(command)
        if self.faulted:
            raise RuntimeError("link is faulted")
        self.record(
            "motion_tx",
            command=command.decode("ascii"),
            tx_hex=command.hex(),
            monotonic_s=self.clock.monotonic(),
        )
        try:
            written = self.transport.write(command)
            self.record(
                "motion_write_returned",
                bytes_written=written,
                expected_bytes=len(command),
                monotonic_s=self.clock.monotonic(),
            )
            if written != len(command):
                raise OSError("motion short write; never retried")
        except (OSError, RuntimeError, ValueError):
            self.faulted = True
            raise

    def attempt_base_stop(self):
        """已知命令最多尝试一次；收到OK也不能证明物理停止。"""
        command = b"#000PDST!"
        receiver = FrameReceiver()
        self.record_best_effort("stop_attempt", command=command.decode("ascii"))
        result = "NO_ACK_UNVERIFIED"
        try:
            # 这是底座000的有限尽力停止请求，不广播，也不因为超时重复发送。
            if self.transport.write(command) != len(command):
                raise OSError("stop short write")
            deadline = self.clock.monotonic() + 0.3
            total = 0
            while self.clock.monotonic() < deadline:
                chunk = self.transport.read(min(self.transport.in_waiting, 256) or 1)
                if not chunk:
                    self.clock.sleep(0.001)
                    continue
                self.record_best_effort("stop_rx", rx_hex=chunk.hex())
                total += len(chunk)
                if total > 4096:
                    break
                if b"#OK!" in receiver.feed(chunk):
                    result = "ACK_RECEIVED_UNVERIFIED"
                    break
        except (OSError, RuntimeError, ValueError) as error:
            result = "SEND_OR_READ_FAILED: " + str(error)
        self.faulted = True
        self.record_best_effort("stop_result", result=result, physical_stop_verified=False)
        return result
