"""ID000局部分方向前馈试验；仅P反馈补偿，不是物理角度标定或全范围模型。

模型来自2026-10-31、T1500的两次方向对照，限定同一固件/模式/局部窗口。
保持原始驱动默认行为不变；不允许饱和后偷偷继续，不自动重发或迭代追赶。
"""

from arm_project.arm_motion import MotionController

POSITIVE_OFFSET_P = 12
NEGATIVE_OFFSET_P = -15
REFERENCE_MIN_P = 1520
REFERENCE_MAX_P = 1543
MODEL_SOURCE = "runs/base_residual_20261031/{negative_fast,positive_fast}/result.json"


def compensated_command(reference_p, start_p):
    if type(reference_p) is not int or not REFERENCE_MIN_P <= reference_p <= REFERENCE_MAX_P:
        raise ValueError("reference outside the two-direction experimental interior [1520, 1543]")
    if type(start_p) is not int or not 1505 <= start_p <= 1555:
        raise ValueError("start outside reviewed base observation window")
    if abs(reference_p - start_p) <= 3:
        return None
    offset = POSITIVE_OFFSET_P if reference_p > start_p else NEGATIVE_OFFSET_P
    command = reference_p + offset
    if not 1505 <= command <= 1555 or abs(command - start_p) > 50:
        raise ValueError("compensation would exceed the reviewed command/step bound")
    return command


class CompensatedBaseController:
    def __init__(self, link, limits, before_motion=None):
        # 复用原驱动的硬件范围及持久化预算校验。
        self.controller = MotionController(link, limits, before_motion=before_motion)
        self.link = link
        self.before_motion = before_motion or (lambda plan: None)
        self.used = False

    def move_reference(self, reference_p):
        # 使用合法示例起点作纯参数校验，拒绝时无串口I/O。
        compensated_command(reference_p, 1530)
        if self.used or self.link.faulted:
            raise RuntimeError("reference session already used or faulted")
        self.used = True
        starts = []
        try:
            for _ in range(3):
                starts.append(self.link.query(0))
                self.controller._wait(0.1)
            values = [s["value"] for s in starts]
            if max(values) - min(values) > 2 or any(not 1505 <= p <= 1555 for p in values):
                raise ValueError("reference preflight position unstable/outside window")
            command = compensated_command(reference_p, values[-1])
            result = {
                "source": self.link.source,
                "reference_p": reference_p,
                "command_p": command,
                "duration_ms": 1500,
                "tolerance_p": 3,
                "model_source": MODEL_SOURCE,
                "model_scope": "LOCAL_EXPERIMENT_ONLY",
                "physical_accuracy_verified": False,
                "initial_samples": starts,
                "command_result": None,
            }
            if command is None:
                reached = all(abs(reference_p - p) <= 3 for p in values)
                result.update(
                    status="REFERENCE_ALREADY_WITHIN_TOLERANCE"
                    if reached
                    else "REFERENCE_NOT_REACHED",
                    reference_reached=reached,
                    final_p=values[-1],
                    reference_error_p=reference_p - values[-1],
                )
                self.link.record("reference_result", result=result)
                return result

            def reserve(plan):
                # 原驱动已再次读固件、模式和3个位置；用最新起点复核补偿方向。
                if (
                    abs(plan["start_p"] - values[-1]) > 2
                    or compensated_command(reference_p, plan["start_p"]) != command
                ):
                    raise ValueError("position/direction changed between preflights")
                enriched = {
                    **plan,
                    "reference_p": reference_p,
                    "offset_p": command - reference_p,
                    "model_source": MODEL_SOURCE,
                }
                self.before_motion(enriched)
                self.link.record("reference_plan", **enriched)

            self.controller.before_motion = reserve
            motion = self.controller.move_joint(
                0, command, 1500, feedback_samples=11, feedback_interval_s=0.5
            )
            result["command_result"] = motion
            feedback = motion["feedback"]
            errors = [reference_p - s["value"] for s in feedback]
            good_status = motion["status"] in ("OBSERVED_NOT_AT_TARGET", "TARGET_REACHED")
            reached = (
                good_status
                and not motion.get("logging_errors")
                and motion["motion_write_complete"]
                and len(errors) == 11
                and all(abs(e) <= 3 for e in errors[-3:])
                and motion.get("tail_span_p", 999) <= 2
            )
            result.update(
                reference_errors_p=errors,
                reference_reached=reached,
                final_p=motion.get("final_p"),
                reference_error_p=errors[-1] if errors else None,
                status="REFERENCE_REACHED" if reached else "REFERENCE_NOT_REACHED",
            )
            if not good_status:
                result["status"] = "REFERENCE_ATTEMPT_FAILED"
            self.link.record_best_effort("reference_result", result=result)
            return result
        except (OSError, RuntimeError, ValueError, KeyboardInterrupt):
            self.link.faulted = True
            raise
