"""PC/ZLink统一局部驱动：显式with连接，精确有限计划，不生成运动轨迹。

profile为已有局部实验窗口，不是整机机械行程。默认构造不打开串口；注入
transport/clock只供离线测试。每个动作先独占保存并fsync预算，再允许精确帧。
cancel仅禁止后续动作，PDST是同轴一次尽力请求，绝不宣称物理停止。
"""

from contextlib import contextmanager
from dataclasses import asdict, dataclass
import json
import math
import os
from pathlib import Path
from threading import Event, Lock

from arm_project.arm_link import Clock, SerialTransport, ServoLink
from arm_project.arm_motion import EXPECTED_FIRMWARE
from arm_project.arm_state import ArmStateReader
from arm_project.arm_tasks import TaughtPose
from arm_project.servo_protocol import (
    make_move_command,
    validate_motion_command,
    validate_read_query,
)


# command min/max, feedback min/max, maximum step, minimum duration.
# 键是舵机地址；六项依次为指令P下/上限、反馈P下/上限、最大单次P差、最短T毫秒。
# 这些是早期已审阅局部窗口，不能作为厂家机械极限；更大整段运动走独立SweepSpec。
PROFILES = {
    0: (1505, 1555, 1505, 1555, 50, 1500),
    1: (1500, 1540, 1480, 1555, 40, 2000),
    2: (1490, 1530, 1470, 1545, 40, 2000),
    3: (1490, 1540, 1475, 1555, 50, 1500),
    4: (1490, 1540, 1470, 1555, 50, 1500),
    5: (1480, 1570, 1460, 1590, 100, 1000),
}


@dataclass(frozen=True)
class MoveSpec:
    # 不可变动作规格：target_p为协议位置值，duration_ms为舵机声明的运动时长。
    servo_id: int
    target_p: int
    duration_ms: int

    def __post_init__(self):
        if type(self.servo_id) is not int or self.servo_id not in PROFILES:
            raise ValueError("servo_id must be an integer in 0..5")
        low, high, _, _, _, minimum = PROFILES[self.servo_id]
        if type(self.target_p) is not int or not low <= self.target_p <= high:
            raise ValueError("target outside the reviewed local command window")
        if type(self.duration_ms) is not int or not minimum <= self.duration_ms <= 5000:
            raise ValueError("duration outside the reviewed local interval")

    @property
    def command(self):
        # 属性只返回ASCII协议字节；真正发送还要经过预算及_PlanGate的精确匹配。
        return make_move_command(self.servo_id, self.target_p, self.duration_ms)


class _RawSerialTransport(SerialTransport):
    """内部裸写由外层精确计划网关限制；不改变旧SerialTransport白名单。"""

    def write(self, command):
        return self.port.write(command)


class _PlanGate:
    def __init__(self, driver, raw):
        # raw只提供物理收发；driver持有已审阅计划和预算，所有写入必须通过本门。
        self.driver, self.raw = driver, raw
        self.source = raw.source
        self.port_name = getattr(raw, "port_name", None)

    def open(self):
        self.raw.open()

    def close(self):
        self.raw.close()

    @property
    def closed(self):
        return self.raw.closed

    @property
    def in_waiting(self):
        return self.raw.in_waiting

    def read(self, count):
        return self.raw.read(count)

    def write(self, command):
        driver = self.driver
        if type(command) is not bytes:
            raise ValueError("wire commands must be exact bytes")
        try:
            # 只读查询不消耗动作帧；其他命令必须是当前动作的同轴停止或准确目标帧。
            validate_read_query(command)
            kind = "query"
        except ValueError:
            active = driver._active_spec
            if active is not None and command == ("#%03dPDST!" % active.servo_id).encode("ascii"):
                if driver._stop_used:
                    raise RuntimeError("same-axis stop allowance already consumed")
                driver._stop_used = True
                kind = "stop"
            else:
                fields = validate_motion_command(command)
                # active是当前计划项，reserved表示预算已落盘，frame_used防止同一目标重发。
                if (
                    active is None
                    or fields != (active.servo_id, active.target_p, active.duration_ms)
                    or not driver._reserved
                    or driver._frame_used
                ):
                    raise ValueError("frame is not the reserved next planned move")
                kind = "motion"
        if kind != "stop":
            driver._check_cancel()
            if driver._faulted:
                raise RuntimeError("session is faulted")
        if kind == "motion":
            # 在尝试写出前就消费单帧许可；短写或通信异常也不能退回许可后自动重试。
            driver._frame_used = True
        values = {"kind": kind, "command": command.decode("ascii"), "tx_hex": command.hex()}
        if kind == "stop":
            driver._record_safe("wire_tx", **values)
        else:
            driver._record("wire_tx", **values)
            # 日志写入和预算fsync期间的取消也不得遗漏。
            driver._check_cancel()
        if kind == "motion":
            # 只有实际进入write才可能需要停止请求；日志/取消失败不额外发PDST。
            driver._active_attempted = True
            driver._motion_attempts += 1
        count = self.raw.write(command)
        returned = {**values, "bytes_written": count, "expected_bytes": len(command)}
        if kind == "stop":
            driver._record_safe("wire_write_returned", **returned)
        else:
            driver._record("wire_write_returned", **returned)
        if count != len(command):
            raise OSError("short wire write; never retransmit")
        return count


class HardwareArmDriver:
    def __init__(
        self,
        port,
        run_dir,
        planned_moves=(),
        *,
        transport=None,
        clock=None,
        response_observation_s=0.0,
        response_poll_interval_s=0.1,
    ):
        # 先冻结有限计划，再创建独占会话目录；构造阶段不打开串口。
        plans = tuple(planned_moves)
        if len(plans) > 20 or any(not isinstance(spec, MoveSpec) for spec in plans):
            raise ValueError("planned_moves must contain at most 20 validated MoveSpec objects")
        if not isinstance(port, str) or not port:
            raise ValueError("an explicit port name is required")
        if type(response_observation_s) not in (int, float) or not 0 <= response_observation_s <= 9:
            raise ValueError("response_observation_s must be finite in0..9")
        if (
            type(response_poll_interval_s) not in (int, float)
            or not 0.1 <= response_poll_interval_s <= 1
        ):
            raise ValueError("response_poll_interval_s must be finite in0.1..1")
        self.run_dir = Path(run_dir)
        self.run_dir.mkdir(parents=True, exist_ok=False)
        self.planned_moves = plans
        self.clock = clock or Clock()
        self.response_observation_s = float(response_observation_s)
        self.response_poll_interval_s = float(response_poll_interval_s)
        self._raw = transport if transport is not None else _RawSerialTransport(port)
        self.source = self._raw.source
        self.transport = _PlanGate(self, self._raw)
        self.link = None
        self._log = None
        self._lock = Lock()
        self._cancel = Event()
        self._entered = self._is_open = self._faulted = False
        self._closed = False
        self._plan_index = self._motion_attempts = 0
        self._active_spec = None
        # 分别记录：预算落盘、动作帧已消费、已进入真实write、同轴停止请求已消费。
        self._reserved = self._frame_used = self._active_attempted = self._stop_used = False
        self._errors = []
        self._actions = []

    @property
    def closed(self):
        return not self._is_open

    def snapshot(self):
        # 这是驱动会话状态，不会为取得快照而再读串口；remaining_count只表示未消费预算。
        errors = list(self._errors)
        if self.link is not None:
            errors.extend(self.link.log_errors)
        return {
            "source": self.source,
            "entered": self._entered,
            "closed": self.closed,
            "faulted": self._faulted,
            "cancel_requested": self._cancel.is_set(),
            "planned_count": len(self.planned_moves),
            "consumed_count": self._plan_index,
            "remaining_count": len(self.planned_moves) - self._plan_index,
            "motion_attempts": self._motion_attempts,
            "response_observation_s": self.response_observation_s,
            "response_poll_interval_s": self.response_poll_interval_s,
            "logging_errors": errors,
            "physical_stop_verified": False,
        }

    def _record(self, event, **values):
        if self._log is None or self._log.closed:
            raise OSError("session log is not available")
        self._log.write(
            json.dumps(
                {
                    "time": self.clock.timestamp(),
                    "monotonic_s": self.clock.monotonic(),
                    "event": event,
                    **values,
                },
                ensure_ascii=False,
            )
            + "\n"
        )
        self._log.flush()

    def _record_safe(self, event, **values):
        try:
            self._record(event, **values)
        except Exception as error:
            self._errors.append("%s: %s" % (event, error))

    def __enter__(self):
        if self._entered or self._closed:
            raise RuntimeError("a driver session cannot be reopened")
        self._entered = True
        try:
            # 外层with独占串口生命周期；ServoLink只通过精确计划网关进行事务收发。
            self._log = (self.run_dir / "events.jsonl").open("x", encoding="utf-8")
            self.link = ServoLink(self.transport, clock=self.clock, record=self._record)
            self.link.__enter__()
            self._is_open = True
            self._check_cancel()
            return self
        except BaseException as error:
            self._faulted = True
            self._errors.append("session entry: " + str(error))
            self.close()
            raise

    def __exit__(self, exc_type, error, traceback):
        if error is not None:
            self._abort(error)
        else:
            self.close()
        return False

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            # 关闭串口是释放主机资源，不代表舵机物理停住或解除扭矩。
            self.transport.close()
            self._is_open = False
            if self._log is not None:
                self._record_safe("closed")
        except BaseException as error:
            self._faulted = True
            self._errors.append("close: " + str(error))
            self._is_open = not bool(getattr(self._raw, "closed", False))
        finally:
            if self._log is not None:
                try:
                    self._log.close()
                except Exception as error:
                    self._errors.append("close log: " + str(error))
            try:
                # session.json使用独占创建，保留动作结果和已消费预算，不能覆盖旧实验记录。
                with (self.run_dir / "session.json").open("x", encoding="utf-8") as out:
                    json.dump(
                        {
                            **self.snapshot(),
                            "planned_moves": [asdict(s) for s in self.planned_moves],
                            "actions": self._actions,
                        },
                        out,
                        ensure_ascii=False,
                        indent=2,
                    )
            except Exception as error:
                self._errors.append("save session: " + str(error))

    def cancel(self):
        # 只设置取消标记；执行线程在发送前/等待中检查，不能把调用返回当作物理急停完成。
        self._cancel.set()
        return self.snapshot()

    def _check_cancel(self):
        # 文件允许另一个CLI请求取消；Event允许当前进程内请求，二者最终走同一检查。
        if (self.run_dir / "cancel.request").exists():
            self._cancel.set()
        if self._cancel.is_set():
            raise InterruptedError("cancel requested; no subsequent motion is allowed")

    def _wait(self, seconds):
        # 按单调时钟等待，最多每20ms检查一次取消；不会用新的查询/动作延长此截止点。
        end = self.clock.monotonic() + seconds
        while True:
            self._check_cancel()
            # One remaining-time read: cancellation/file I/O or scheduling may
            # cross the deadline between two monotonic() calls.
            # 只读取一次remaining再判断和sleep，避免两次取时跨过截止点后传入负等待值。
            remaining = end - self.clock.monotonic()
            if remaining <= 0:
                break
            self.clock.sleep(min(0.02, remaining))
        self._check_cancel()

    @contextmanager
    def _operation(self):
        # 非阻塞锁保证同一驱动同时只有一个高层操作；内部*_locked方法复用已经取得的锁。
        if not self._lock.acquire(blocking=False):
            raise RuntimeError("another driver operation is running")
        try:
            if not self._is_open or self._closed or self._faulted:
                raise RuntimeError("driver session is not active")
            yield
        finally:
            self._lock.release()

    def _stop(self):
        # 只有确实尝试过的当前轴才有一次PDST预算；从未发运动时不额外制造停止流量。
        if self._active_spec is None or not self._active_attempted or self._stop_used:
            return None
        try:
            command = ("#%03dPDST!" % self._active_spec.servo_id).encode("ascii")
            written = self.transport.write(command)
            return {
                "servo_id": self._active_spec.servo_id,
                "bytes_written": written,
                "physical_stop_verified": False,
                "ack_verified": False,
            }
        except BaseException as error:
            return {"error": str(error), "physical_stop_verified": False}

    def _abort(self, error):
        # 尽力停止后锁定并关闭整个会话；不能在未知执行结果下继续下一动作或自动回零。
        stop = self._stop()
        self._faulted = True
        self._cancel.set()
        if self.link is not None:
            self.link.faulted = True
        if self._log is not None and not self._log.closed:
            self._record_safe("session_fault", error=str(error), stop_attempt=stop)
        self.close()
        return stop

    def _read_state_locked(self):
        self._check_cancel()
        state = ArmStateReader(self.link).read_state()
        self._check_cancel()
        if state["status"] != "OK":
            self._abort(RuntimeError(state.get("error") or "incomplete six-axis state"))
        return state

    def read_state(self):
        with self._operation():
            try:
                return self._read_state_locked()
            except BaseException as error:
                self._abort(error)
                raise

    def _consume(self, spec, kind, execution_id, step_id):
        self._check_cancel()
        if (
            self._plan_index >= len(self.planned_moves)
            or self.planned_moves[self._plan_index] != spec
        ):
            raise ValueError("move is not the next item in the finite planned_moves sequence")
        number = self._plan_index + 1
        path = self.run_dir / ("attempt_%02d.json" % number)
        # 先独占创建第number次预算文件并fsync，再允许一条完全匹配的运动帧写出。
        with path.open("x", encoding="utf-8") as out:
            self._plan_index += 1  # 失败/取消也不恢复预算。
            json.dump(
                {
                    "time": self.clock.timestamp(),
                    "kind": kind,
                    "plan_index": number,
                    "execution_id": execution_id,
                    "step_id": step_id,
                    "spec": asdict(spec),
                    "command": spec.command.decode("ascii"),
                },
                out,
                ensure_ascii=False,
                indent=2,
            )
            out.flush()
            os.fsync(out.fileno())
        self._check_cancel()
        self._reserved = True

    def _move_locked(self, servo_id, target_p, duration_ms, execution_id, step_id):
        # response_pass记录是否有合方向且稳定的反馈；position_reached另按原±3P标准核对。
        # 两个标志都不等于实测末端坐标准确，夹爪P反馈也不证明夹到了物体。
        result = {
            "source": self.source,
            "execution_id": execution_id,
            "step_id": step_id,
            "servo_id": servo_id,
            "target_p": target_p,
            "duration_ms": duration_ms,
            "status": "FAILED",
            "response_pass": False,
            "position_reached": False,
            "motion_attempted": False,
            "motion_write_complete": False,
            "physical_stop_verified": False,
            "start_samples": [],
            "feedback": [],
            "error": None,
        }
        self._active_spec = None
        self._reserved = self._frame_used = self._active_attempted = self._stop_used = False
        try:
            self._check_cancel()
            if self.link.log_errors or self._errors:
                raise RuntimeError("session logging has failed")
            spec = MoveSpec(servo_id, target_p, duration_ms)
            if (
                self._plan_index >= len(self.planned_moves)
                or spec != self.planned_moves[self._plan_index]
            ):
                raise ValueError("wrong move order or exhausted planned_moves")
            self._active_spec = spec
            _, _, fb_low, fb_high, max_step, _ = PROFILES[servo_id]
            # 先核对当前固件与模式，不能把历史初始化状态当作本次动作的依据。
            version = self.link.query(servo_id, "version")["value"]
            self._wait(0.1)
            mode = self.link.query(servo_id, "mode")["value"]
            self._wait(0.1)
            if version != EXPECTED_FIRMWARE or mode != 1:
                raise ValueError("firmware/mode does not match the reviewed setup")
            for _ in range(3):
                result["start_samples"].append(self.link.query(servo_id))
                self._wait(0.1)
            starts = [s["value"] for s in result["start_samples"]]
            # 三次新鲜起点应稳定，且均位于反馈窗口、距目标不超过本局部计划的最大P差。
            if max(starts) - min(starts) > 2:
                raise ValueError("fresh starting feedback is unstable")
            if any(not fb_low <= p <= fb_high or abs(target_p - p) > max_step for p in starts):
                raise ValueError("fresh position/window or maximum step check failed")
            start = starts[-1]
            result["start_p"] = start
            no_motion = all(abs(target_p - p) <= 3 for p in starts)
            self._consume(
                spec, "NO_MOTION_NEEDED" if no_motion else "MOTION", execution_id, step_id
            )
            if no_motion:
                # 目标已在误差带内就只做最终查询；仍消费本计划项，避免下次重复使用预算。
                final = self.link.query(servo_id)
                self._check_cancel()
                p = final["value"]
                if (
                    not fb_low <= p <= fb_high
                    or abs(target_p - p) > 3
                    or max(starts + [p]) - min(starts + [p]) > 2
                ):
                    raise ValueError("no-motion final verification left the stable target window")
                result.update(
                    status="SUCCEEDED",
                    response_pass=True,
                    position_reached=True,
                    no_motion_needed=True,
                    final_p=p,
                    error_p=target_p - p,
                    final_verification=final,
                )
            else:
                self._check_cancel()
                self.link.send_motion(spec.command)
                # Host write-return time, not a servo acknowledgement or physical arrival.
                # 该时刻是主机write返回，不是舵机确认或物理到位时间。
                result["motion_write_monotonic_s"] = self.clock.monotonic()
                result["motion_attempted"] = True
                result["motion_write_complete"] = True
                self._wait(duration_ms / 1000.0 + 0.3)
                # direction把增P和减P统一为正向位移，下面用乘积识别反向反馈。
                direction = 1 if target_p > start else -1
                for index in range(11):
                    self._check_cancel()
                    sample = self.link.query(servo_id)
                    result["feedback"].append(sample)
                    p = sample["value"]
                    if not fb_low <= p <= fb_high or (p - start) * direction < -2:
                        raise ValueError("feedback outside local window or opposite to the command")
                    if index < 10:
                        self._wait(0.1)
                # Explicit experimental opt-in: observe longer, never resend a
                # motion. Default0 preserves the original response interval.
                # 可选延长“只读观察”时间，不重复发送运动；缺省0保留原来的观察时段。
                deadline = result["motion_write_monotonic_s"] + self.response_observation_s
                while self.clock.monotonic() < deadline:
                    now = self.clock.monotonic()
                    # Only reduce traffic in the middle; retain dense final0.5s.
                    # 中段可降低查询频率，末0.5秒仍密集采样，以判断末尾反馈是否稳定。
                    interval = self.response_poll_interval_s if now < deadline - 0.5 else 0.1
                    next_at = min(deadline, now + interval)
                    if now < deadline - 0.5:
                        next_at = min(next_at, deadline - 0.5)
                    self._wait(max(0.0, next_at - now))
                    sample = self.link.query(servo_id)
                    self._check_cancel()
                    result["feedback"].append(sample)
                    p = sample["value"]
                    if not fb_low <= p <= fb_high or (p - start) * direction < -2:
                        raise ValueError("feedback outside local window or opposite to the command")
                values = [sample["value"] for sample in result["feedback"]]
                # 有明显合方向变化、末5点稳定后才检查最终值；只变化0–2P仍视为无明显响应。
                if (values[-1] - start) * direction <= 2:
                    raise ValueError("no distinct directional response; cancel remaining moves")
                if max(values[-5:]) - min(values[-5:]) > 2:
                    raise ValueError("post-motion tail is not stable")
                self._wait(0.1)
                final = self.link.query(servo_id)
                self._check_cancel()
                p = final["value"]
                if (
                    not fb_low <= p <= fb_high
                    or (p - start) * direction < -2
                    or abs(p - values[-1]) > 2
                ):
                    raise ValueError("final verification left the stable local response")
                reached = (
                    all(abs(target_p - pv) <= 3 for pv in values[-3:]) and abs(target_p - p) <= 3
                )
                # 有响应但未满足±3P时保留RESPONSE_OBSERVED，不冒充严格到位成功。
                result.update(
                    response_pass=True,
                    position_reached=reached,
                    final_p=p,
                    error_p=target_p - p,
                    final_verification=final,
                    observed_delta_p=p - start,
                    tail_span_p=max(values[-5:]) - min(values[-5:]),
                    status="SUCCEEDED" if reached else "RESPONSE_OBSERVED",
                )
            self._record("move_result", result=result)
            self._actions.append(result)
            return result
        except BaseException as error:
            result.update(
                status="CANCELLED"
                if isinstance(error, (InterruptedError, KeyboardInterrupt))
                else "FAILED",
                error=type(error).__name__ + ": " + str(error),
                response_pass=False,
                position_reached=False,
                motion_attempted=self._active_attempted,
            )
            self._actions.append(result)
            result["stop_attempt"] = self._abort(error)
            return result
        finally:
            self._active_spec = None
            self._reserved = False

    def move_joint(self, servo_id, target_p, duration_ms, *, execution_id="", step_id=""):
        with self._operation():
            return self._move_locked(servo_id, target_p, duration_ms, execution_id, step_id)

    def sample_joint_at(self, servo_id, deadlines_s):
        """Read one joint at bounded host-monotonic deadlines; never send motion.

        Missed deadlines, cancellation, communication errors or out-of-window
        readings fault the session and prevent remaining planned moves. The
        samples retain actual reception times; scheduling is not real-time.
        """
        if type(servo_id) is not int or servo_id not in PROFILES:
            raise ValueError("servo_id must be an integer in0..5")
        if not isinstance(deadlines_s, (list, tuple)) or not 1 <= len(deadlines_s) <= 61:
            raise ValueError("provide1..61 explicit sample deadlines")
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in deadlines_s):
            raise ValueError("deadlines must be finite monotonic times")
        if any(b - a < 0.05 for a, b in zip(deadlines_s, deadlines_s[1:])):
            raise ValueError("sample deadlines must increase by at least0.05s")
        with self._operation():
            try:
                now = self.clock.monotonic()
                if deadlines_s[0] < now - 0.2 or deadlines_s[-1] > now + 15:
                    raise ValueError("sample schedule must start now and end within15s")
                _, _, low, high, _, _ = PROFILES[servo_id]
                result = {"servo_id": servo_id, "source": self.source, "samples": []}
                for deadline in deadlines_s:
                    # deadline是预定主机采样时刻；query/reception另存实际时刻，不能伪装实时同步。
                    self._wait(max(0.0, deadline - self.clock.monotonic()))
                    queried_at = self.clock.monotonic()
                    if queried_at - deadline > 0.2:
                        raise RuntimeError("sample query missed its deadline by more than0.2s")
                    sample = self.link.query(servo_id)
                    self._check_cancel()
                    row = {
                        "deadline_monotonic_s": deadline,
                        "query_monotonic_s": queried_at,
                        **sample,
                    }
                    result["samples"].append(row)
                    self._record("scheduled_position_sample", servo_id=servo_id, **row)
                    if not low <= sample["value"] <= high:
                        raise ValueError("observed position outside reviewed local feedback window")
                    if sample["received_monotonic_s"] - deadline > 0.2:
                        raise RuntimeError("sample reception missed its deadline by more than0.2s")
                result["status"] = "OK"
                self._record("scheduled_observation", result=result)
                return result
            except BaseException as error:
                self._abort(error)
                raise

    def grip(self, action, *, execution_id="", step_id=""):
        with self._operation():
            if action not in ("open", "close"):
                error = ValueError("grip action must be open or close")
                self._abort(error)
                raise error
            result = self._move_locked(
                5, 1480 if action == "open" else 1570, 1000, execution_id, step_id
            )
            # 这里只复用005的局部开合动作；负载和夹持均不能由舵机反馈推断。
            # object_grasped=False且grasp_verified=False表示未证实夹持，不等于证实空载。
            result.update(
                grip_action=action,
                object_grasped=False,
                grasp_verified=False,
                meaning="local gripper response only; load and grasp not inferred",
            )
            return result

    def capture_pose(self, name):
        with self._operation():
            try:
                if not isinstance(name, str) or not name.strip():
                    raise ValueError("pose needs a name")
                states = []
                # 采三次完整六轴快照做稳定性比较；每次内部仍按地址顺序读，不是同步锁存。
                for index in range(3):
                    state = self._read_state_locked()
                    if state["status"] != "OK":
                        raise RuntimeError("incomplete pose capture")
                    states.append(state)
                    if index < 2:
                        self._wait(0.1)
                for axis in range(6):
                    values = [s["servos"][axis]["position_p"] for s in states]
                    if max(values) - min(values) > 2:
                        raise ValueError("pose capture is not stable on all six axes")
                final = states[-1]
                # 保存的是实际反馈P数组；verified=False要求之后另行审核姿态及路径。
                data = {
                    "name": name,
                    "positions_p": [s["position_p"] for s in final["servos"]],
                    "source": self.source,
                    "captured_at": final["finished_at"],
                    "verified": False,
                }
                self._record("pose_capture", pose=data, states=states)
                return TaughtPose.from_dict(data) if self.source == "HARDWARE" else data
            except BaseException as error:
                self._abort(error)
                raise

    def move_pose(self, pose, *, execution_id, step_id, path_review=None):
        with self._operation():
            result = {
                "source": self.source,
                "execution_id": execution_id,
                "step_id": step_id,
                "status": "FAILED",
                "position_reached": False,
                "response_pass": False,
                "segments": [],
                "error": None,
                "physical_stop_verified": False,
            }
            try:
                self._check_cancel()
                if not isinstance(pose, TaughtPose) or not pose.verified:
                    raise ValueError("move_pose requires a verified taught pose")
                review = path_review
                if (
                    not isinstance(review, dict)
                    or review.get("reviewed") is not True
                    or review.get("pose_name") != pose.name
                    or not isinstance(review.get("evidence"), str)
                    or not review["evidence"].strip()
                ):
                    raise ValueError("explicit reviewed path and evidence are required")
                starts = review.get("start_positions_p")
                if (
                    not isinstance(starts, (list, tuple))
                    or len(starts) != 6
                    or any(type(p) is not int or not 500 <= p <= 2500 for p in starts)
                ):
                    raise ValueError("path review needs six raw starting P values")
                rows = review.get("segments")
                # 路径是显式有限的逐轴动作序列，不把一个六轴目标数组自动插值成轨迹。
                if not isinstance(rows, (list, tuple)) or not 1 <= len(rows) <= 20:
                    raise ValueError("path must explicitly list its finite sequential segments")
                specs = tuple(row if isinstance(row, MoveSpec) else MoveSpec(**row) for row in rows)
                if any(spec.servo_id == 5 for spec in specs):
                    raise ValueError("move_pose cannot move the gripper")
                if specs != self.planned_moves[self._plan_index : self._plan_index + len(specs)]:
                    raise ValueError("path differs from the next finite planned sequence")
                ending = list(starts)
                # 离线先推演顺序动作的最终P，确认前五轴终点确为所请求姿态，夹爪另行控制。
                for spec in specs:
                    ending[spec.servo_id] = spec.target_p
                if ending[:5] != list(pose.positions_p[:5]):
                    raise ValueError("reviewed path does not end at the requested five-axis pose")
                current = self._read_state_locked()
                # 发首条动作前重新核对六轴当前反馈，避免从错误/陈旧起点执行旧路径。
                if current["status"] != "OK" or any(
                    abs(s["position_p"] - starts[i]) > 3 for i, s in enumerate(current["servos"])
                ):
                    raise ValueError("current six-axis state differs from reviewed path start")
                for index, spec in enumerate(specs):
                    segment = self._move_locked(
                        spec.servo_id,
                        spec.target_p,
                        spec.duration_ms,
                        execution_id,
                        "%s/%02d" % (step_id, index),
                    )
                    result["segments"].append(segment)
                    # 严格姿态接口要求本段SUCCEEDED才进入下一段，与普通驱动响应验收分开。
                    if segment["status"] != "SUCCEEDED":
                        raise RuntimeError(
                            "path segment did not strictly reach its target; no subsequent segment"
                        )
                final = self._read_state_locked()
                result["final_state"] = final
                if final["status"] != "OK":
                    raise RuntimeError("final pose state incomplete")
                positions = [s["position_p"] for s in final["servos"]]
                if (
                    any(abs(positions[i] - pose.positions_p[i]) > 3 for i in range(5))
                    or abs(positions[5] - starts[5]) > 3
                ):
                    raise ValueError("final five-axis pose or unchanged gripper hold check failed")
                result.update(
                    status="SUCCEEDED",
                    response_pass=True,
                    position_reached=True,
                    physical_path_accuracy_verified=False,
                )
                self._record("pose_result", result=result)
                return result
            except BaseException as error:
                result.update(
                    status="CANCELLED"
                    if isinstance(error, (InterruptedError, KeyboardInterrupt))
                    else "FAILED",
                    response_pass=False,
                    position_reached=False,
                    error=type(error).__name__ + ": " + str(error),
                )
                self._abort(error)
                return result
