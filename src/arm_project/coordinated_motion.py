"""Reviewed same-duration joint targets, one port and one finite batch.

Targets are written once in sequence, then all six joints are read periodically.
Host write skew is recorded; this does not claim synchronous firmware starts or
a Cartesian straight line. Camera, IK, grasp proof and geometry are separate.
"""

from dataclasses import asdict, dataclass
import math
from pathlib import Path
import re

from arm_project.arm_link import SerialTransport
from arm_project.arm_motion import EXPECTED_FIRMWARE
from arm_project.arm_state import ArmStateReader
from arm_project.joint_probe import JointProbeController
from arm_project.joint_sweep import SweepSpec
from arm_project.servo_protocol import validate_read_query


@dataclass(frozen=True)
class CoordinatedSpec:
    review_id: str
    whole_path_reviewed_for_motion: bool
    evidence: str
    axes: tuple
    held_max_drift_p: int = 10
    response_tail_tolerance_p: int = 2

    def __post_init__(self):
        if type(self.review_id) is not str or not re.fullmatch(
            r"[A-Za-z0-9_-]{1,80}", self.review_id
        ):
            raise ValueError("invalid bounded review_id")
        if (
            self.whole_path_reviewed_for_motion is not True
            or type(self.evidence) is not str
            or not self.evidence.strip()
        ):
            raise ValueError("review the combined whole path, not only individual endpoints")
        if (
            not isinstance(self.axes, (list, tuple))
            or not 1 <= len(self.axes) <= 5
            or any(not isinstance(a, SweepSpec) for a in self.axes)
        ):
            raise ValueError("one..five validated axis sweeps required")
        object.__setattr__(self, "axes", tuple(self.axes))
        ids = [a.servo_id for a in self.axes]
        if len(ids) != len(set(ids)) or 5 in ids:
            raise ValueError("distinct body axes000..004 only; gripper is separate")
        policies = {(a.duration_ms, a.sample_interval_s, a.settling_s) for a in self.axes}
        if len(policies) != 1 or self.axes[0].sample_interval_s < 0.5:
            raise ValueError("same duration and bounded>=0.5s observation policy required")
        if type(self.held_max_drift_p) is not int or not 1 <= self.held_max_drift_p <= 20:
            raise ValueError("explicit held-axis observation tolerance required")
        if (
            type(self.response_tail_tolerance_p) is not int
            or not 2 <= self.response_tail_tolerance_p <= 10
        ):
            raise ValueError("response tail bound must be an explicit integer2..10P")

    @classmethod
    def from_dict(cls, data):
        fields = {
            "review_id",
            "whole_path_reviewed_for_motion",
            "evidence",
            "axes",
            "held_max_drift_p",
            "response_tail_tolerance_p",
        }
        required = fields - {"response_tail_tolerance_p"}
        if (
            not isinstance(data, dict)
            or set(data) - fields
            or required - set(data)
            or not isinstance(data["axes"], list)
        ):
            raise ValueError("provide a reviewed group specification, not a nominal proposal")
        return cls(**{**data, "axes": tuple(SweepSpec.from_dict(a) for a in data["axes"])})


class _RawSerial(SerialTransport):
    def write(self, command):
        return self.port.write(command)


class CoordinatedTransport:
    """Only next exact frame after reservation; stops only for attempted axes."""

    def __init__(self, port, spec, *, raw=None):
        if not isinstance(spec, CoordinatedSpec):
            raise ValueError("validated coordinated spec required")
        self.spec = spec
        self.raw = _RawSerial(port) if raw is None else raw
        self.port_name = port
        self.source = self.raw.source
        self.armed = self.cancelled = False
        self.index = 0
        self.attempted_ids = []
        self.stop_used = set()

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

    def arm_after_reservation(self):
        if self.armed or self.cancelled or self.index:
            raise RuntimeError("batch already armed, cancelled or consumed")
        self.armed = True

    def write(self, command):
        if type(command) is not bytes:
            raise ValueError("exact bytes required")
        try:
            validate_read_query(command)
        except ValueError:
            sid = next(
                (i for i in self.attempted_ids if command == f"#{i:03d}PDST!".encode("ascii")), None
            )
            if sid is not None:
                if sid in self.stop_used:
                    raise RuntimeError("stop allowance consumed")
                self.stop_used.add(sid)
                self.cancelled = True
            else:
                if not self.armed or self.cancelled or self.index >= len(self.spec.axes):
                    raise RuntimeError("no reserved next motion")
                spec = self.spec.axes[self.index]
                if command != spec.command:
                    raise ValueError("not the next exact planned frame")
                self.index += 1
                self.attempted_ids.append(spec.servo_id)
        return self.raw.write(command)


class CoordinatedController(JointProbeController):
    """Reuse bounded wait/cancel helpers; no intermediate motion targets."""

    def __init__(self, link, spec, before_motion=None, *, cancel_file=None):
        if (
            not isinstance(spec, CoordinatedSpec)
            or not isinstance(link.transport, CoordinatedTransport)
            or link.transport.spec != spec
        ):
            raise ValueError("matching reviewed group transport required")
        if link.source == "HARDWARE" and not callable(before_motion):
            raise ValueError("hardware group requires a persistent batch reservation")
        if before_motion is not None and not callable(before_motion):
            raise ValueError("invalid reservation callback")
        self.link, self.spec = link, spec
        self.before_motion = before_motion or (lambda plan: None)
        self.cancel_file = Path(cancel_file) if cancel_file is not None else None
        self.used = self.cancel_requested = False

    def _check_cancel(self):
        if self.cancel_file is not None and self.cancel_file.exists():
            self.cancel_requested = True
        super()._check_cancel()

    def _stop_group(self):
        results = []
        for sid in self.link.transport.attempted_ids:
            if sid in self.link.transport.stop_used:
                continue
            command = f"#{sid:03d}PDST!".encode("ascii")
            item = {
                "servo_id": sid,
                "command": command.decode("ascii"),
                "physical_stop_verified": False,
                "ack_verified": False,
            }
            self.link.record_best_effort("group_stop_attempt", **item)
            try:
                n = self.link.transport.write(command)
                item["bytes_written"] = n
                if n != len(command):
                    item["error"] = "short stop write"
            except Exception as error:
                item["error"] = type(error).__name__ + ": " + str(error)
            self.link.record_best_effort("group_stop_result", **item)
            results.append(item)
        return results

    def run(self):
        if self.used or self.link.faulted or self.link.log_errors:
            raise RuntimeError("group used, faulted or missing reliable logs")
        self.used = True
        result = {
            "source": self.link.source,
            "spec": asdict(self.spec),
            "status": "PREFLIGHT",
            "initial_state": None,
            "start_samples": {},
            "samples": [],
            "write_times": [],
            "axes": [],
            "response_pass": False,
            "position_reached": False,
            "motion_attempted": False,
            "physical_task_verified": False,
            "physical_stop_verified": False,
            "stops": [],
            "error": None,
        }
        try:
            self._check_cancel()
            reader = ArmStateReader(self.link)
            initial = reader.read_state()
            result["initial_state"] = initial
            if initial["status"] != "OK":
                raise ValueError("incomplete starting arm state")
            baseline = {s["servo_id"]: s["position_p"] for s in initial["servos"]}
            starts = {}
            for axis in self.spec.axes:
                self._check_cancel()
                v = self.link.query(axis.servo_id, "version")
                self._wait(0.05)
                mode = self.link.query(axis.servo_id, "mode")
                if v["value"] != EXPECTED_FIRMWARE or mode["value"] != 1:
                    raise ValueError("firmware/position mode differs from review")
                samples = []
                for _ in range(3):
                    self._wait(0.1)
                    samples.append(self.link.query(axis.servo_id))
                result["start_samples"][str(axis.servo_id)] = samples
                ps = [s["value"] for s in samples]
                if max(ps) - min(ps) > 2 or any(
                    not axis.start_min_p <= p <= axis.start_max_p for p in ps
                ):
                    raise ValueError("unstable or outside reviewed fresh starting interval")
                if any(abs(axis.target_p - p) > axis.max_step_p for p in ps):
                    raise ValueError("fresh step exceeds review")
                starts[axis.servo_id] = ps[-1]
            self._check_cancel()
            self.before_motion(
                {
                    "spec": asdict(self.spec),
                    "start_p": starts,
                    "commands": [a.command.decode("ascii") for a in self.spec.axes],
                }
            )
            self._check_cancel()
            self.link.transport.arm_after_reservation()
            for axis in self.spec.axes:
                self._check_cancel()
                result["motion_attempted"] = True
                self.link.send_motion(axis.command)
                result["write_times"].append(
                    {"servo_id": axis.servo_id, "returned_monotonic_s": self.link.clock.monotonic()}
                )
            timestamps = [w["returned_monotonic_s"] for w in result["write_times"]]
            result["host_write_skew_ms"] = (max(timestamps) - min(timestamps)) * 1000
            written_at = max(timestamps)
            policy = self.spec.axes[0]
            total = policy.duration_ms / 1000 + policy.settling_s
            moving = {a.servo_id: a for a in self.spec.axes}
            for i in range(1, math.ceil(total / policy.sample_interval_s) + 1):
                due = min(i * policy.sample_interval_s, total)
                self._wait(max(0, written_at + due - self.link.clock.monotonic()))
                state = reader.read_state()
                sample = {
                    "scheduled_elapsed_s": due,
                    "elapsed_at_finish_s": self.link.clock.monotonic() - written_at,
                    "state": state,
                }
                result["samples"].append(sample)
                self.link.record("coordinated_sample", **sample)
                self._check_cancel()
                if state["status"] != "OK":
                    raise ValueError("incomplete coordinated feedback")
                if sample["elapsed_at_finish_s"] > total + 0.75:
                    raise ValueError("coordinated observation deadline exceeded")
                for item in state["servos"]:
                    sid, p = item["servo_id"], item["position_p"]
                    if sid in moving:
                        axis = moving[sid]
                        if not axis.min_p <= p <= axis.max_p:
                            raise ValueError("active feedback outside reviewed window")
                        if (
                            abs(axis.target_p - starts[sid]) > 3
                            and (p - starts[sid]) * (1 if axis.target_p > starts[sid] else -1) < -2
                        ):
                            raise ValueError("active feedback opposite to requested direction")
                    elif abs(p - baseline[sid]) > self.spec.held_max_drift_p:
                        raise ValueError("uncommanded axis drift exceeded reviewed tolerance")
            for axis in self.spec.axes:
                sid = axis.servo_id
                values = [
                    next(s["position_p"] for s in row["state"]["servos"] if s["servo_id"] == sid)
                    for row in result["samples"]
                ]
                tail = values[-3:]
                stable = max(tail) - min(tail) <= 2
                response_settled = max(tail) - min(tail) <= self.spec.response_tail_tolerance_p
                reached = stable and all(abs(axis.target_p - p) <= 3 for p in tail)
                delta = values[-1] - starts[sid]
                directed = delta * (axis.target_p - starts[sid]) > 0 and abs(delta) > 2
                near_start = abs(axis.target_p - starts[sid]) <= 3
                result["axes"].append(
                    {
                        "servo_id": sid,
                        "start_p": starts[sid],
                        "target_p": axis.target_p,
                        "final_p": values[-1],
                        "error_p": axis.target_p - values[-1],
                        "tail_span_p": max(tail) - min(tail),
                        "strict_settled": stable,
                        "response_tail_within_review": response_settled,
                        "motion_observed": directed,
                        "response_pass": response_settled
                        and (directed or (near_start and reached)),
                        "position_reached": reached,
                    }
                )
            result["response_pass"] = all(a["response_pass"] for a in result["axes"])
            result["position_reached"] = all(a["position_reached"] for a in result["axes"])
            if not result["response_pass"]:
                raise ValueError("at least one axis lacks stable directional response")
            result["status"] = (
                "TARGET_REACHED" if result["position_reached"] else "RESPONSE_OBSERVED"
            )
        except (KeyboardInterrupt, InterruptedError) as error:
            result.update(status="CANCELLED", error=str(error) or "interrupted")
        except Exception as error:
            result.update(
                status="FAILED_AFTER_ATTEMPT" if result["motion_attempted"] else "PREFLIGHT_FAILED",
                error=type(error).__name__ + ": " + str(error),
            )
        finally:
            if result["error"]:
                result["response_pass"] = result["position_reached"] = False
                result["stops"] = self._stop_group()
                self.link.faulted = True
            self.link.record_best_effort("coordinated_result", result=result)
            if self.link.log_errors:
                result.update(
                    status="LOGGING_FAILED",
                    error="coordinated logging incomplete",
                    response_pass=False,
                    position_reached=False,
                )
                result["stops"] += self._stop_group()
                self.link.faulted = True
            result["logging_errors"] = list(self.link.log_errors)
        return result
