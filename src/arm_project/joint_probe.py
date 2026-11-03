"""逐关节、单条指令的受限探测模块；导入不会打开设备或执行动作。

ProbeSpec中的review/evidence是调用者提供的本次实物审阅记录，不自动构成
授权，也不等于全行程标定。每个其他关节必须独立填写实际审阅的窗口。
本模块不更改原SerialTransport的底座限制、不改模式、不释力或自动回程。
"""

from dataclasses import asdict, dataclass

from arm_project.arm_link import SerialTransport
from arm_project.arm_motion import EXPECTED_FIRMWARE
from arm_project.servo_protocol import FrameReceiver, make_move_command, validate_read_query


def _int(value, name, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ValueError("%s must be an integer in [%d, %d]" % (name, low, high))


@dataclass(frozen=True)
class ProbeSpec:
    review_id: str
    reviewed_for_motion: bool
    evidence: str
    servo_id: int
    min_p: int
    max_p: int
    start_min_p: int
    start_max_p: int
    target_p: int
    duration_ms: int
    max_step_p: int
    tolerance_p: int = 3

    def __post_init__(self):
        for name in ("review_id", "evidence"):
            value = getattr(self, name)
            if type(value) is not str or not value.strip():
                raise ValueError(name + " must be a nonempty review record")
        if self.reviewed_for_motion is not True:
            raise ValueError("this exact motion must have a completed review")
        _int(self.servo_id, "servo_id", 0, 5)
        for name in ("min_p", "max_p", "start_min_p", "start_max_p", "target_p"):
            _int(getattr(self, name), name, 500, 2500)
        if not self.min_p < self.max_p:
            raise ValueError("reviewed window must have distinct ordered endpoints")
        if not self.min_p <= self.start_min_p <= self.start_max_p <= self.max_p:
            raise ValueError("starting interval must be inside the reviewed window")
        if not self.min_p <= self.target_p <= self.max_p:
            raise ValueError("target must be inside the reviewed window")
        # 一次明确审阅的夹爪时长对照；不降低其他轴/其他规格的默认下限。
        timing_exception = (
            self.review_id == "gripper_timing_1000_20261031_01"
            and self.servo_id == 5
            and self.target_p == 1450
            and self.min_p == 1435
            and self.max_p == 1510
            and self.max_step_p == 50
            and self.duration_ms == 1000
        )
        _int(self.duration_ms, "duration_ms", 1000 if timing_exception else 1500, 5000)
        _int(self.max_step_p, "max_step_p", 1, 50)
        _int(self.tolerance_p, "tolerance_p", 3, 3)

    @classmethod
    def from_dict(cls, data):
        if not isinstance(data, dict):
            raise ValueError("probe specification must be a dictionary")
        fields = set(cls.__dataclass_fields__)
        required = fields - {"tolerance_p"}
        if set(data) - fields or required - set(data):
            raise ValueError("probe specification has unknown or missing fields")
        return cls(**data)

    @property
    def command(self):
        return make_move_command(self.servo_id, self.target_p, self.duration_ms)

    @property
    def stop_command(self):
        return ("#%03dPDST!" % self.servo_id).encode("ascii")


class ExactProbeTransport(SerialTransport):
    """新会话只放行spec精确目标一次；与原底座专用后端分开。

    运动与停止的额度都在调用底层write前消耗；短写/异常不恢复额度。
    独立的持久化预算仍由上层before_motion负责，重新实例化不能代替授权。
    """

    def __init__(self, port_name, spec):
        if not isinstance(spec, ProbeSpec):
            raise ValueError("ExactProbeTransport requires a validated ProbeSpec")
        super().__init__(port_name)
        self.spec = spec
        self.motion_attempted = False
        self.stop_attempted = False

    def write(self, command):
        if type(command) is not bytes:
            raise ValueError("command must be exact bytes")
        try:
            validate_read_query(command)
        except ValueError:
            if command == self.spec.command:
                if self.motion_attempted:
                    raise RuntimeError("this exact probe has consumed its motion allowance")
                self.motion_attempted = True
            elif command == self.spec.stop_command:
                if self.stop_attempted:
                    raise RuntimeError("this exact probe has consumed its stop allowance")
                self.stop_attempted = True
                # 先停止再启动也不允许；已取消的精确探测不能借原额度重开。
                self.motion_attempted = True
            else:
                raise ValueError("command is not a whitelisted query or the reviewed exact probe")
        return self.port.write(command)


class JointProbeController:
    """run()执行一次；cancel()阻止后续动作，PDST仅为未验证的尽力停止。"""

    def __init__(self, link, spec, before_motion=None, *, feedback_samples=11):
        if not isinstance(spec, ProbeSpec):
            raise ValueError("JointProbeController requires a validated ProbeSpec")
        _int(feedback_samples, "feedback_samples", 11, 61)
        if link.source == "HARDWARE":
            if not callable(before_motion):
                raise ValueError("hardware probe requires a persistent motion-budget callback")
            if not isinstance(link.transport, ExactProbeTransport) or link.transport.spec != spec:
                raise ValueError("hardware link must use the matching exact-probe transport")
        elif before_motion is not None and not callable(before_motion):
            raise ValueError("before_motion must be callable")
        self.link = link
        self.spec = spec
        self.before_motion = before_motion or (lambda plan: None)
        self.used = False
        self.cancel_requested = False
        self.stop_attempted = False
        self.feedback_samples = feedback_samples

    def cancel(self):
        self.cancel_requested = True

    def _check_cancel(self):
        if self.cancel_requested:
            raise InterruptedError("probe cancellation requested")

    def _wait(self, seconds):
        deadline = self.link.clock.monotonic() + seconds
        while True:
            self._check_cancel()
            remaining = deadline - self.link.clock.monotonic()
            if remaining <= 0:
                break
            self.link.clock.sleep(min(0.02, remaining))
        self._check_cancel()

    def _attempt_stop(self):
        if self.stop_attempted:
            return "ALREADY_ATTEMPTED_UNVERIFIED"
        self.stop_attempted = True
        command = self.spec.stop_command
        receiver = FrameReceiver()
        self.link.record_best_effort(
            "stop_attempt", servo_id=self.spec.servo_id, command=command.decode("ascii")
        )
        status = "NO_ACK_UNVERIFIED"
        try:
            written = self.link.transport.write(command)
            self.link.record_best_effort(
                "stop_write_returned", bytes_written=written, expected_bytes=len(command)
            )
            if written != len(command):
                raise OSError("probe stop short write")
            deadline = self.link.clock.monotonic() + 0.3
            total = 0
            while self.link.clock.monotonic() < deadline:
                chunk = self.link.transport.read(min(self.link.transport.in_waiting, 256) or 1)
                if not chunk:
                    self.link.clock.sleep(0.001)
                    continue
                self.link.record_best_effort("stop_rx", rx_hex=chunk.hex())
                total += len(chunk)
                if total > 4096:
                    break
                if b"#OK!" in receiver.feed(chunk):
                    status = "ACK_RECEIVED_UNVERIFIED"
                    break
        except Exception as error:
            status = "SEND_OR_READ_FAILED: %s: %s" % (type(error).__name__, error)
        finally:
            self.link.faulted = True
            self.link.record_best_effort(
                "stop_result",
                servo_id=self.spec.servo_id,
                result=status,
                physical_stop_verified=False,
            )
        return status

    def run(self):
        if self.used or self.link.faulted or self.link.log_errors:
            raise RuntimeError("probe session already used, faulted, or missing reliable logs")
        self.used = True
        spec = self.spec
        result = {
            "source": self.link.source,
            "review_id": spec.review_id,
            "spec": asdict(spec),
            "servo_id": spec.servo_id,
            "target_p": spec.target_p,
            "duration_ms": spec.duration_ms,
            "tolerance_p": 3,
            "required_consecutive_samples": 3,
            "feedback_samples_requested": self.feedback_samples,
            "status": "PREFLIGHT",
            "motion_attempted": False,
            "motion_write_complete": False,
            "physical_stop_verified": False,
            "physical_motion_independently_verified": False,
            "stop_attempt": None,
            "start_samples": [],
            "feedback": [],
            "target_reached": False,
            "response_pass": False,
            "error": None,
        }
        try:
            self._check_cancel()
            version = self.link.query(spec.servo_id, "version")
            self._wait(0.1)
            mode = self.link.query(spec.servo_id, "mode")
            result.update(version=version, mode=mode)
            if version["value"] != EXPECTED_FIRMWARE or mode["value"] != 1:
                raise ValueError("firmware/position mode differs from reviewed setup")
            self._wait(0.1)
            for _ in range(3):
                result["start_samples"].append(self.link.query(spec.servo_id))
                self._wait(0.1)
            starts = [sample["value"] for sample in result["start_samples"]]
            if any(
                type(p) is not int or not spec.start_min_p <= p <= spec.start_max_p for p in starts
            ):
                raise ValueError("starting feedback outside the reviewed starting interval")
            if max(starts) - min(starts) > 2:
                raise ValueError("starting feedback is not stable within 2P")
            start = starts[-1]
            result.update(start_p=start, start_span_p=max(starts) - min(starts))
            if any(abs(spec.target_p - p) > spec.max_step_p for p in starts):
                raise ValueError("probe step exceeds reviewed bound")
            if abs(spec.target_p - start) <= 3:
                if not all(abs(spec.target_p - p) <= 3 for p in starts):
                    raise ValueError("near-target preflight lacks three in-tolerance samples")
                result["status"] = "NO_MOTION_NEEDED"
                return result
            self._check_cancel()
            plan = {**asdict(spec), "start_p": start, "command": spec.command.decode("ascii")}
            self.before_motion(plan)
            self.link.record("probe_plan", **plan)
            self._check_cancel()
            result["motion_attempted"] = True
            self.link.send_motion(spec.command)
            result["motion_write_complete"] = True
            self._wait(spec.duration_ms / 1000.0 + 0.3)
            consecutive = 0
            for index in range(self.feedback_samples):
                self._check_cancel()
                sample = self.link.query(spec.servo_id)
                sample["error_p"] = spec.target_p - sample["value"]
                result["feedback"].append(sample)
                p = sample["value"]
                if type(p) is not int or not spec.min_p <= p <= spec.max_p:
                    raise ValueError("feedback left the reviewed observation window")
                direction = 1 if spec.target_p > start else -1
                if (p - start) * direction < -2:
                    raise ValueError("feedback changed opposite to the requested direction")
                consecutive = consecutive + 1 if abs(sample["error_p"]) <= 3 else 0
                if index < self.feedback_samples - 1:
                    self._wait(0.5)
            values = [sample["value"] for sample in result["feedback"]]
            tail_span = max(values[-5:]) - min(values[-5:])
            reached = consecutive >= 3 and tail_span <= 2
            result.update(
                final_p=values[-1],
                error_p=spec.target_p - values[-1],
                observed_delta_p=values[-1] - start,
                feedback_span_p=max(values) - min(values),
                tail_span_p=tail_span,
                directed_feedback_change=(values[-1] - start) * (spec.target_p - start) > 0
                and abs(values[-1] - start) > max(2, max(starts) - min(starts)),
                target_reached=reached,
                stable_residual_observed=tail_span <= 2 and not reached,
                observation_span_s=result["feedback"][-1]["received_monotonic_s"]
                - result["feedback"][0]["received_monotonic_s"],
                status="TARGET_REACHED" if reached else "OBSERVED_NOT_AT_TARGET",
            )
            result["response_pass"] = result["directed_feedback_change"] and tail_span <= 2
            if not reached and not result["directed_feedback_change"]:
                result["stable_residual_observed"] = False
                raise ValueError("no distinct directional response; end probe without retry")
        except (KeyboardInterrupt, InterruptedError) as error:
            result["status"] = (
                "CANCELLED_STOP_UNVERIFIED"
                if result["motion_attempted"]
                else "CANCELLED_BEFORE_MOTION"
            )
            result["error"] = str(error) or "keyboard interrupt"
            if result["motion_attempted"]:
                result["stop_attempt"] = self._attempt_stop()
            self.link.faulted = True
        except Exception as error:
            result["status"] = (
                "FAILED_AFTER_ATTEMPT" if result["motion_attempted"] else "PREFLIGHT_FAILED"
            )
            result["error"] = "%s: %s" % (type(error).__name__, error)
            if result["motion_attempted"]:
                result["stop_attempt"] = self._attempt_stop()
            self.link.faulted = True
        finally:
            if result["error"]:
                result["response_pass"] = False
            self.link.record_best_effort("probe_result", result=result)
            if self.link.log_errors:
                if result["error"] is None:
                    result["error"] = "probe logging did not complete"
                    result["status"] = (
                        "FAILED_AFTER_ATTEMPT" if result["motion_attempted"] else "PREFLIGHT_FAILED"
                    )
                if result["motion_attempted"] and result["stop_attempt"] is None:
                    result["stop_attempt"] = self._attempt_stop()
                self.link.faulted = True
                result["target_reached"] = False
                result["response_pass"] = False
            result["logging_errors"] = list(self.link.log_errors)
        return result
