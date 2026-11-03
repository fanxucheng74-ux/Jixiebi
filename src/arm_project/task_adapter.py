"""Reviewed, finite task integration. Import/compile never opens a device.

Compile first, then construct HardwareArmDriver with compiled.planned_moves.
The adapter never upgrades RESPONSE_OBSERVED to SUCCEEDED. A close operation
requires a fresh external observation; simulation reports remain SIMULATION.
"""

from copy import deepcopy
from dataclasses import dataclass
from datetime import datetime, timezone
from threading import Event, Lock
from types import MappingProxyType
from uuid import uuid4

from arm_project.arm_tasks import ArmTaskRunner, MOVE_PHASES
from arm_project.unified_arm import MoveSpec, PROFILES


def _aware(value):
    if isinstance(value, str):
        value = datetime.fromisoformat(value[:-1] + "+00:00" if value.endswith("Z") else value)
    if not isinstance(value, datetime) or value.tzinfo is None or value.utcoffset() is None:
        raise ValueError("an offset-aware timestamp is required")
    return value.astimezone(timezone.utc)


class _ValidationCallbacks:
    def move_pose(self, *args, **kwargs):
        raise AssertionError("static validation cannot execute a pose")

    def grip(self, *args, **kwargs):
        raise AssertionError("static validation cannot operate the gripper")


@dataclass(frozen=True)
class ReviewedPath:
    # 一条已审核路径绑定到step_id，而不只是pose_name；同一姿态可能从不同起点进入。
    step_id: str
    pose_name: str
    evidence: str
    start_positions_p: tuple
    segments: tuple

    def driver_review(self):
        # Return a fresh container: caller changes cannot mutate the compiled plan.
        return {
            "reviewed": True,
            "pose_name": self.pose_name,
            "evidence": self.evidence,
            "start_positions_p": list(self.start_positions_p),
            "segments": list(self.segments),
        }


@dataclass(frozen=True)
class CompiledTask:
    poses: object
    steps: tuple
    paths: object
    planned_moves: tuple


def compile_task(poses, steps, path_reviews):
    """Validate all eight phases and expand the exact <=20-command budget.

    path_reviews is keyed by step ID, e.g. 00:approach and 04:lift, even when
    both phases refer to the same pose. Every starting six-P vector is supplied
    explicitly; no trajectory, grasp point or measured gripper position is invented.
    HARDWARE pose metadata is not an authenticity check: it must come from reviewed
    capture records. Tests may use explicitly labelled in-memory fixtures only.
    """
    # 先离线展开整趟任务的精确运动清单，再交给驱动；本函数不会打开设备。
    poses, steps = dict(poses), tuple(steps)
    ArmTaskRunner(_ValidationCallbacks(), poses, steps)  # complete existing task validation
    if not isinstance(path_reviews, dict):
        raise ValueError("path_reviews must be a dictionary keyed by step_id")
    required = {"%02d:%s" % (i, s.name) for i, s in enumerate(steps) if s.name in MOVE_PHASES}
    if set(path_reviews) != required:
        raise ValueError("exactly one reviewed path is required for every move step")
    paths, planned = {}, []
    previous_arm = None
    previous_gripper = None
    for index, step in enumerate(steps):
        step_id = "%02d:%s" % (index, step.name)
        if step.kind == "grip":
            spec = MoveSpec(5, 1480 if step.action == "open" else 1570, 1000)
            if (
                previous_gripper is not None
                and abs(spec.target_p - previous_gripper) > PROFILES[5][4]
            ):
                raise ValueError("gripper transition exceeds the reviewed maximum step")
            planned.append(spec)
            # Only a consistency constraint for the next explicit review. Actual
            # feedback is still checked by the driver; this is not a measurement.
            previous_gripper = spec.target_p
            continue
        review = path_reviews[step_id]
        if (
            not isinstance(review, dict)
            or review.get("reviewed") is not True
            or review.get("pose_name") != step.pose_name
            or not isinstance(review.get("evidence"), str)
            or not review["evidence"].strip()
        ):
            raise ValueError("each path needs a matching pose, explicit review and evidence")
        start = review.get("start_positions_p")
        if (
            not isinstance(start, (tuple, list))
            or len(start) != 6
            or any(type(p) is not int or not 500 <= p <= 2500 for p in start)
        ):
            raise ValueError("each reviewed start needs six raw integer P values")
        # 六P数组下标就是舵机地址；前五项是臂身，第六项是夹爪。
        start = tuple(start)
        if any(not PROFILES[i][2] <= p <= PROFILES[i][3] for i, p in enumerate(start)):
            raise ValueError("reviewed start lies outside a currently allowed feedback window")
        if previous_arm is not None and tuple(start[:5]) != previous_arm:
            raise ValueError("reviewed arm paths do not form a continuous explicit sequence")
        if previous_gripper is not None and abs(start[5] - previous_gripper) > 3:
            raise ValueError("reviewed gripper hold contradicts the preceding successful step")
        rows = review.get("segments")
        if not isinstance(rows, (list, tuple)) or not 1 <= len(rows) <= 20:
            raise ValueError("each reviewed path needs 1..20 explicit segments")
        segments = tuple(row if isinstance(row, MoveSpec) else MoveSpec(**row) for row in rows)
        # ending只模拟每段目标变化，用于检查段间衔接，不是读取到的真实位置。
        ending = list(start)
        for spec in segments:
            if spec.servo_id == 5:
                raise ValueError("pose paths cannot contain gripper commands")
            if abs(spec.target_p - ending[spec.servo_id]) > PROFILES[spec.servo_id][4]:
                raise ValueError("a reviewed path segment exceeds the maximum local step")
            ending[spec.servo_id] = spec.target_p
        if tuple(ending[:5]) != poses[step.pose_name].positions_p[:5]:
            raise ValueError("reviewed path does not end at its named five-axis pose")
        paths[step_id] = ReviewedPath(step_id, step.pose_name, review["evidence"], start, segments)
        planned.extend(segments)
        previous_arm = tuple(ending[:5])
        previous_gripper = start[5]
    if len(planned) > 20:
        raise ValueError("the complete task exceeds the 20-command session budget")
    return CompiledTask(MappingProxyType(poses), steps, MappingProxyType(paths), tuple(planned))


class ReviewedTaskAdapter:
    """Compose a precompiled task with an explicitly connected unified driver.

    observer(context) -> {object_grasped: True, evidence: {...}} is optional.
    Evidence needs EXTERNAL_OBSERVATION, matching execution_id/step_id/observation_id,
    an observed_at within this close callback, and a nonempty description. A boolean,
    a stale record, actuator feedback, or the lower driver's flag alone is insufficient.
    The observer is responsible for collecting genuine observations and returning
    promptly; this synchronous API cannot forcibly interrupt arbitrary user callbacks.
    """

    def __init__(self, driver, compiled, *, observer=None, now=None):
        if not isinstance(compiled, CompiledTask):
            raise ValueError("compile_task must complete before constructing the adapter")
        # Recompile to reject hand-constructed or modified CompiledTask containers.
        checked = compile_task(
            compiled.poses,
            compiled.steps,
            {key: value.driver_review() for key, value in compiled.paths.items()},
        )
        if checked.planned_moves != compiled.planned_moves:
            raise ValueError("compiled budget does not match its reviewed paths")
        if tuple(getattr(driver, "planned_moves", ())) != checked.planned_moves:
            raise ValueError("driver must be constructed with the complete exact compiled budget")
        if any(
            not callable(getattr(driver, name, None))
            for name in ("move_pose", "grip", "cancel", "close")
        ):
            raise ValueError("driver needs move_pose, grip, cancel and close")
        if getattr(driver, "source", None) not in ("HARDWARE", "SIMULATION"):
            raise ValueError("driver must explicitly identify its source")
        if observer is not None and not callable(observer):
            raise ValueError("observer must be callable")
        self.driver, self.compiled = driver, checked
        self.source = driver.source
        self.observer = observer
        self.now = now or (lambda: datetime.now(timezone.utc))
        self._cancel = Event()
        self._lock = Lock()
        self._execution_id = None
        self._next_index = 0
        self._blocked = False
        self._runner_created = False
        self.cleanup_errors = []

    def create_runner(self):
        if self._runner_created:
            raise RuntimeError("a task adapter may create only one task session")
        self._runner_created = True
        return ReviewedTaskSession(self)

    def cancel(self):
        # No close from a second thread: the running callback/driver owns cleanup.
        self._cancel.set()
        self._blocked = True
        try:
            self.driver.cancel()
        except Exception as error:
            self.cleanup_errors.append("cancel: " + str(error))

    def _halt(self):
        self.cancel()
        try:
            self.driver.close()
        except Exception as error:
            self.cleanup_errors.append("close: " + str(error))

    def _begin(self, kind, value, execution_id, step_id):
        # 只接受编译清单中的下一次回调，拒绝换任务、跳步或临时替换目标。
        if self._cancel.is_set() or self._blocked:
            raise InterruptedError("task adapter is cancelled or failed")
        if not isinstance(execution_id, str) or not execution_id:
            raise ValueError("execution_id is required")
        if self._execution_id is None:
            self._execution_id = execution_id
        if execution_id != self._execution_id or self._next_index >= len(self.compiled.steps):
            raise ValueError("wrong execution or exhausted task steps")
        step = self.compiled.steps[self._next_index]
        expected = "%02d:%s" % (self._next_index, step.name)
        if step_id != expected or step.kind != kind:
            raise ValueError("callback is not the next compiled task step")
        if (kind == "move" and value != self.compiled.poses[step.pose_name]) or (
            kind == "grip" and value != step.action
        ):
            raise ValueError("callback differs from the compiled pose or gripper action")
        self._next_index += 1  # An attempted callback cannot be retried in this session.

    def _check_result(self, result, execution_id, step_id):
        # source区分硬件与模拟结果；身份、响应和严格到位都要核对。
        if (
            not isinstance(result, dict)
            or result.get("execution_id") != execution_id
            or result.get("step_id") != step_id
            or result.get("source") != self.source
        ):
            raise ValueError("driver result identity/source does not match the current callback")
        if result.get("status") != "SUCCEEDED":
            self._halt()
            return False
        if result.get("response_pass") is not True or result.get("position_reached") is not True:
            raise ValueError("driver success lacks strict response and position confirmation")
        return True

    def _observe(self, result, execution_id, step_id, started):
        # observer是独立观察回调的引用；每次生成新observation_id防止旧证据重用。
        # Never trust an object's-grasped flag produced only by the actuator driver.
        result.update(object_grasped=False, grasp_verified=False)
        result.pop("evidence", None)
        if self.observer is None:
            result["grasp_evidence_error"] = "no external observer is configured"
            self._halt()
            return
        observation_id = uuid4().hex
        context = {
            "execution_id": execution_id,
            "step_id": step_id,
            "observation_id": observation_id,
            "started_at": started.isoformat(),
            "source": self.source,
            "driver_result": deepcopy(result),
        }
        observed = self.observer(context)
        finished = _aware(self.now())
        if self._cancel.is_set():
            raise InterruptedError("cancelled during external observation")
        evidence = observed.get("evidence") if isinstance(observed, dict) else None
        if (
            not isinstance(observed, dict)
            or observed.get("object_grasped") is not True
            or not isinstance(evidence, dict)
        ):
            raise ValueError("external observation needs explicit object evidence, not a boolean")
        if evidence.get("source") != "EXTERNAL_OBSERVATION":
            raise ValueError("actuator feedback is not external grasp evidence")
        if any(
            evidence.get(key) != context[key]
            for key in ("execution_id", "step_id", "observation_id")
        ):
            raise ValueError("observation belongs to a different execution, step or challenge")
        timestamp = _aware(evidence.get("observed_at"))
        if not started <= timestamp <= finished:
            raise ValueError("observation was not recorded during the current close callback")
        if not isinstance(evidence.get("description"), str) or not evidence["description"].strip():
            raise ValueError("external observation needs a description")
        result.update(
            object_grasped=True,
            grasp_verified=True,
            evidence={**deepcopy(evidence), "observed_at": timestamp.isoformat()},
        )

    def _execute(self, kind, value, execution_id, step_id):
        # 此处连接任务与驱动：校验当前步骤→调用驱动→核对回包→必要时取得抓取证据。
        if not self._lock.acquire(blocking=False):
            raise RuntimeError("a task adapter callback is already running")
        result = {
            "source": self.source,
            "execution_id": execution_id,
            "step_id": step_id,
            "status": "FAILED",
            "response_pass": False,
            "position_reached": False,
            "object_grasped": False,
            "physical_task_verified": False,
        }
        try:
            self._begin(kind, value, execution_id, step_id)
            started = _aware(self.now())
            if self._cancel.is_set():
                raise InterruptedError("cancelled before the driver callback")
            if kind == "move":
                result = deepcopy(
                    self.driver.move_pose(
                        value,
                        execution_id=execution_id,
                        step_id=step_id,
                        path_review=self.compiled.paths[step_id].driver_review(),
                    )
                )
            else:
                result = deepcopy(
                    self.driver.grip(value, execution_id=execution_id, step_id=step_id)
                )
            if self._cancel.is_set():
                raise InterruptedError("cancelled during the driver callback")
            if (
                self._check_result(result, execution_id, step_id)
                and kind == "grip"
                and value == "close"
            ):
                try:
                    self._observe(result, execution_id, step_id, started)
                except ValueError as error:
                    result.update(
                        object_grasped=False, grasp_verified=False, grasp_evidence_error=str(error)
                    )
                    result.pop("evidence", None)
                    self._halt()
            result["physical_task_verified"] = False
            return result
        except BaseException as error:
            if not isinstance(result, dict):
                result = {}
            result.update(
                source=self.source,
                execution_id=execution_id,
                step_id=step_id,
                status="CANCELLED"
                if isinstance(error, (InterruptedError, KeyboardInterrupt))
                else "FAILED",
                response_pass=False,
                position_reached=False,
                object_grasped=False,
                grasp_verified=False,
                physical_task_verified=False,
                error=type(error).__name__ + ": " + str(error),
            )
            result.pop("evidence", None)
            self._halt()
            return result
        finally:
            self._lock.release()

    def move_pose(self, pose, *, execution_id, step_id):
        return self._execute("move", pose, execution_id, step_id)

    def grip(self, action, *, execution_id, step_id):
        return self._execute("grip", action, execution_id, step_id)


class ReviewedTaskSession:
    """Runner composition whose public cancel reaches both task and driver."""

    def __init__(self, adapter):
        self.adapter = adapter
        self._runner = ArmTaskRunner(
            adapter, adapter.compiled.poses, adapter.compiled.steps, now=adapter.now
        )

    def snapshot(self):
        return {
            **self._runner.snapshot(),
            "source": self.adapter.source,
            "cleanup_errors": list(self.adapter.cleanup_errors),
            "physical_task_verified": False,
        }

    def start(self):
        self._runner.start()
        return self.snapshot()

    def tick(self):
        state = self._runner.tick()
        if state["state"] not in ("READY", "RUNNING", "SUCCEEDED"):
            self.adapter._halt()
        return self.snapshot()

    def cancel(self):
        self._runner.cancel()
        self.adapter.cancel()
        return self.snapshot()
