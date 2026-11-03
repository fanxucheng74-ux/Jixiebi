"""One continuous position command over a separately reviewed whole sweep.

This is a different policy from small-step discovery. It preserves the old local
probe limits and never streams intermediate targets, torque/configuration writes,
or automatic retries. P-to-angle/Cartesian calibration remains external.
"""

from dataclasses import asdict, dataclass
import math
from pathlib import Path

from arm_project.arm_link import SerialTransport
from arm_project.arm_motion import EXPECTED_FIRMWARE
from arm_project.joint_probe import ExactProbeTransport, JointProbeController, _int
from arm_project.servo_protocol import make_move_command


@dataclass(frozen=True)
class SweepSpec:
    review_id: str
    whole_path_reviewed_for_motion: bool
    evidence: str
    servo_id: int
    min_p: int
    max_p: int
    start_min_p: int
    start_max_p: int
    target_p: int
    duration_ms: int
    max_step_p: int
    sample_interval_s: float = 0.5
    settling_s: float = 2.0

    def __post_init__(self):
        for name in ("review_id", "evidence"):
            if type(getattr(self, name)) is not str or not getattr(self, name).strip():
                raise ValueError(name + " needs a nonempty review record")
        if self.whole_path_reviewed_for_motion is not True:
            raise ValueError("review the whole swept path before a continuous command")
        _int(self.servo_id, "servo_id", 0, 5)
        for name in ("min_p", "max_p", "start_min_p", "start_max_p", "target_p"):
            _int(getattr(self, name), name, 500, 2500)
        if not self.min_p < self.max_p:
            raise ValueError("invalid observation window")
        if not self.min_p <= self.start_min_p <= self.start_max_p <= self.max_p:
            raise ValueError("start interval outside reviewed window")
        if not self.min_p <= self.target_p <= self.max_p:
            raise ValueError("target outside reviewed window")
        # Protocol bounds, not a mechanical full-range authorization. The review
        # supplies this particular axis/path/duration and the exact one-shot target.
        _int(self.duration_ms, "duration_ms", 1500, 9999)
        _int(self.max_step_p, "max_step_p", 1, 2000)
        if (
            max(abs(self.target_p - p) for p in (self.start_min_p, self.start_max_p))
            > self.max_step_p
        ):
            raise ValueError("reviewed start/target exceeds the sweep step bound")
        for name, low, high in (("sample_interval_s", 0.2, 1.0), ("settling_s", 1.0, 3.0)):
            value = getattr(self, name)
            if (
                type(value) not in (int, float)
                or not math.isfinite(value)
                or not low <= value <= high
            ):
                raise ValueError(name + " outside bounded observation policy")

    @classmethod
    def from_dict(cls, data):
        if not isinstance(data, dict):
            raise ValueError("sweep needs an object")
        fields = set(cls.__dataclass_fields__)
        required = fields - {"sample_interval_s", "settling_s"}
        if set(data) - fields or required - set(data):
            raise ValueError("unknown or missing sweep fields")
        return cls(**data)

    @property
    def command(self):
        return make_move_command(self.servo_id, self.target_p, self.duration_ms)

    @property
    def stop_command(self):
        return ("#%03dPDST!" % self.servo_id).encode("ascii")


class ExactSweepTransport(ExactProbeTransport):
    """Reuse the exact-frame, one-write gate with a separately validated policy."""

    def __init__(self, port_name, spec):
        if not isinstance(spec, SweepSpec):
            raise ValueError("continuous sweep transport requires SweepSpec")
        SerialTransport.__init__(self, port_name)
        self.spec = spec
        self.motion_attempted = False
        self.stop_attempted = False


class JointSweepController(JointProbeController):
    """Reuse cancellation/wait/one-axis stop; collect feedback during one move."""

    def __init__(self, link, spec, before_motion=None, *, cancel_file=None):
        if not isinstance(spec, SweepSpec):
            raise ValueError("continuous controller requires SweepSpec")
        if link.source == "HARDWARE":
            if not callable(before_motion):
                raise ValueError("hardware sweep needs a persistent one-shot budget")
            if not isinstance(link.transport, ExactSweepTransport) or link.transport.spec != spec:
                raise ValueError("hardware sweep needs its matching exact transport")
        elif before_motion is not None and not callable(before_motion):
            raise ValueError("before_motion must be callable")
        self.link, self.spec = link, spec
        self.before_motion = before_motion or (lambda plan: None)
        self.cancel_file = Path(cancel_file) if cancel_file is not None else None
        self.used = self.cancel_requested = self.stop_attempted = False

    def _check_cancel(self):
        if self.cancel_file is not None and self.cancel_file.exists():
            self.cancel_requested = True
        super()._check_cancel()

    def run(self):
        if self.used or self.link.faulted or self.link.log_errors:
            raise RuntimeError("sweep session used, faulted or missing logs")
        self.used = True
        spec = self.spec
        result = {
            "source": self.link.source,
            "policy": "ONE_CONTINUOUS_POSITION_COMMAND",
            "spec": asdict(spec),
            "status": "PREFLIGHT",
            "start_samples": [],
            "feedback": [],
            "motion_attempted": False,
            "motion_write_complete": False,
            "response_pass": False,
            "position_reached": False,
            "physical_stop_verified": False,
            "physical_angle_verified": False,
            "stop_attempt": None,
            "error": None,
        }
        try:
            self._check_cancel()
            result["version"] = self.link.query(spec.servo_id, "version")
            self._wait(0.1)
            result["mode"] = self.link.query(spec.servo_id, "mode")
            if result["version"]["value"] != EXPECTED_FIRMWARE or result["mode"]["value"] != 1:
                raise ValueError("firmware/mode differs from the reviewed setup")
            for _ in range(3):
                self._wait(0.1)
                result["start_samples"].append(self.link.query(spec.servo_id))
            starts = [s["value"] for s in result["start_samples"]]
            if any(not spec.start_min_p <= p <= spec.start_max_p for p in starts):
                raise ValueError("fresh starting feedback outside reviewed interval")
            if max(starts) - min(starts) > 2:
                raise ValueError("fresh starting feedback unstable")
            if any(abs(spec.target_p - p) > spec.max_step_p for p in starts):
                raise ValueError("fresh start exceeds reviewed sweep distance")
            start = result["start_p"] = starts[-1]
            if all(abs(spec.target_p - p) <= 3 for p in starts):
                result.update(status="NO_MOTION_NEEDED", position_reached=True)
                return result
            self._check_cancel()
            plan = {**asdict(spec), "start_p": start, "command": spec.command.decode("ascii")}
            self.before_motion(plan)
            self.link.record("sweep_plan", **plan)
            self._check_cancel()
            result["motion_attempted"] = True
            self.link.send_motion(spec.command)
            result["motion_write_complete"] = True
            written_at = self.link.clock.monotonic()
            result["write_returned_monotonic_s"] = written_at
            duration_s = spec.duration_ms / 1000
            total_s = duration_s + spec.settling_s
            count = math.ceil(total_s / spec.sample_interval_s)
            for index in range(1, count + 1):
                due = min(index * spec.sample_interval_s, total_s)
                self._wait(max(0.0, written_at + due - self.link.clock.monotonic()))
                sample = self.link.query(spec.servo_id)
                elapsed = sample["received_monotonic_s"] - written_at
                sample = {
                    **sample,
                    "scheduled_elapsed_s": due,
                    "elapsed_s": elapsed,
                    "after_command_duration": elapsed > duration_s,
                }
                result["feedback"].append(sample)
                self.link.record("sweep_sample", **sample)
                self._check_cancel()
                if elapsed > total_s + 0.5:
                    raise ValueError("continuous observation exceeded its deadline")
                p = sample["value"]
                if not spec.min_p <= p <= spec.max_p:
                    raise ValueError("feedback outside reviewed whole-sweep window")
                direction = 1 if spec.target_p > start else -1
                if (p - start) * direction < -2:
                    raise ValueError("feedback opposite to the reviewed direction")
            values = [s["value"] for s in result["feedback"]]
            tail = values[-3:]
            delta = values[-1] - start
            directed = delta * (spec.target_p - start) > 0 and abs(delta) > 2
            stable = max(tail) - min(tail) <= 2
            reached = stable and all(abs(spec.target_p - p) <= 3 for p in tail)
            result.update(
                final_p=values[-1],
                error_p=spec.target_p - values[-1],
                observed_delta_p=delta,
                tail_span_p=max(tail) - min(tail),
                directed_feedback_change=directed,
                response_pass=directed and stable,
                position_reached=reached,
                samples_during_declared_motion=sum(
                    not s["after_command_duration"] for s in result["feedback"]
                ),
                status="TARGET_REACHED" if reached else "RESPONSE_OBSERVED",
            )
            if not directed or not stable:
                raise ValueError("no distinct stable directional response; no retry")
        except (KeyboardInterrupt, InterruptedError) as error:
            result["status"] = "CANCELLED"
            result["error"] = str(error) or "keyboard interrupt"
        except Exception as error:
            result["status"] = (
                "FAILED_AFTER_ATTEMPT" if result["motion_attempted"] else "PREFLIGHT_FAILED"
            )
            result["error"] = type(error).__name__ + ": " + str(error)
        finally:
            if result["error"]:
                result["response_pass"] = result["position_reached"] = False
                if result["motion_attempted"]:
                    result["stop_attempt"] = self._attempt_stop()
                self.link.faulted = True
            self.link.record_best_effort("sweep_result", result=result)
            if self.link.log_errors:
                result.update(
                    status="LOGGING_FAILED",
                    error="sweep logging incomplete",
                    response_pass=False,
                    position_reached=False,
                )
                if result["motion_attempted"] and result["stop_attempt"] is None:
                    result["stop_attempt"] = self._attempt_stop()
                self.link.faulted = True
            result["logging_errors"] = list(self.link.log_errors)
        return result
