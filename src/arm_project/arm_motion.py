"""受限运动验收：一次下发、有界反馈观察，不重试、不自动回程。"""

import json
from pathlib import Path

from arm_project.servo_protocol import make_move_command


EXPECTED_FIRMWARE = "ZL-ZServo_AD_CBM V2.1.16STG"


def load_limits(path):
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if data.get("schema_version") != 1:
        raise ValueError("unsupported limits schema")
    rows = data.get("joints", [])
    if len(rows) != 6 or [r.get("servo_id") for r in rows] != list(range(6)):
        raise ValueError("limits must describe IDs 0 to 5 in order")
    for row in rows:
        if type(row["servo_id"]) is not int or type(row.get("motion_enabled")) is not bool:
            raise ValueError("invalid ID or enable flag")
        if row["motion_enabled"]:
            names = ("min_p", "max_p", "max_step_p", "min_duration_ms", "max_duration_ms")
            if any(type(row.get(name)) is not int for name in names):
                raise ValueError("enabled joint needs integer limits")
            if not (
                500 <= row["min_p"] < row["max_p"] <= 2500
                and 0 < row["max_step_p"] <= row["max_p"] - row["min_p"]
                and 20 <= row["min_duration_ms"] <= row["max_duration_ms"] <= 9999
            ):
                raise ValueError("invalid limit ranges")
    return rows


class MotionController:
    def __init__(self, link, limits, before_motion=None):
        if link.source == "HARDWARE":
            if not callable(before_motion):
                raise ValueError("hardware requires a persistent one-attempt budget callback")
            base = limits[0]
            if (
                not 1505 <= base["min_p"] < base["max_p"] <= 1555
                or not 0 < base["max_step_p"] <= 50
                or not 1500 <= base["min_duration_ms"] <= base["max_duration_ms"] <= 5000
            ):
                raise ValueError("configuration cannot widen the reviewed hardware envelope")
        self.link = link
        self.limits = limits
        self.before_motion = before_motion or (lambda plan: None)
        self.used = False
        self.cancel_requested = False

    def cancel(self):
        self.cancel_requested = True

    def _wait(self, seconds):
        deadline = self.link.clock.monotonic() + seconds
        while self.link.clock.monotonic() < deadline:
            if self.cancel_requested:
                raise InterruptedError("cancel requested")
            self.link.clock.sleep(min(0.02, deadline - self.link.clock.monotonic()))

    def move_joint(
        self, servo_id, target_p, duration_ms=1500, *, feedback_samples=5, feedback_interval_s=0.1
    ):
        # 先检查全部参数，拒绝时不发送任何查询或运动。
        if type(feedback_samples) is not int or not 5 <= feedback_samples <= 61:
            raise ValueError("feedback_samples must be an integer in [5, 61]")
        if (
            type(feedback_interval_s) not in (int, float)
            or not 0.1 <= feedback_interval_s <= 1.0
            or (feedback_samples - 1) * feedback_interval_s > 30
        ):
            raise ValueError("feedback interval/window outside bounded observation policy")
        command = make_move_command(servo_id, target_p, duration_ms)
        row = self.limits[servo_id]
        if not row["motion_enabled"]:
            raise ValueError("joint motion disabled pending calibration")
        if servo_id != 0:
            raise ValueError("V1 hardware acceptance only supports base ID000")
        if not row["min_p"] <= target_p <= row["max_p"]:
            raise ValueError("target outside configured command window")
        if not row["min_duration_ms"] <= duration_ms <= row["max_duration_ms"]:
            raise ValueError("duration outside configured limits")
        if self.used or self.link.faulted:
            raise RuntimeError("session has already used or lost its motion allowance")
        result = {
            "source": self.link.source,
            "status": "PREFLIGHT",
            "servo_id": servo_id,
            "target_p": target_p,
            "duration_ms": duration_ms,
            "tolerance_p": 3,
            "required_consecutive_samples": 3,
            "feedback_samples_requested": feedback_samples,
            "feedback_interval_s": feedback_interval_s,
            "motion_attempted": False,
            "motion_write_complete": False,
            "physical_stop_verified": False,
            "stop_attempt": None,
            "start_samples": [],
            "feedback": [],
            "error": None,
        }
        try:
            if self.cancel_requested:
                raise InterruptedError("cancel requested before motion")
            version = self.link.query(servo_id, "version")["value"]
            self._wait(0.1)
            mode = self.link.query(servo_id, "mode")["value"]
            if version != EXPECTED_FIRMWARE or mode != 1:
                raise ValueError("firmware or position mode differs from verified hardware")
            self._wait(0.1)
            for _ in range(3):
                result["start_samples"].append(self.link.query(servo_id))
                # 包括最后一次查询后的间隔，沿用此前可工作的串行时序。
                self._wait(0.1)
            starts = [sample["value"] for sample in result["start_samples"]]
            if max(starts) - min(starts) > 2:
                raise ValueError("starting position not stable within 2P")
            start = starts[-1]
            result["start_p"] = start
            if any(not row["min_p"] <= p <= row["max_p"] for p in starts):
                raise ValueError("starting position outside reviewed command window")
            if abs(target_p - start) > row["max_step_p"]:
                raise ValueError("step exceeds configured bound")
            if abs(target_p - start) <= result["tolerance_p"]:
                result["status"] = "NO_MOTION_NEEDED"
                return result
            if self.cancel_requested:
                raise InterruptedError("cancel requested before write")
            plan = {
                "servo_id": servo_id,
                "start_p": start,
                "target_p": target_p,
                "duration_ms": duration_ms,
                "command": command.decode("ascii"),
            }
            # 先持久化授权使用，再写串口；短写或异常也不得重新发运动。
            self.used = True
            self.before_motion(plan)
            self.link.record("motion_plan", **plan)
            if self.cancel_requested:
                raise InterruptedError("cancel requested while persisting the motion plan")
            result["motion_attempted"] = True
            self.link.send_motion(command)
            result["motion_write_complete"] = True
            self._wait(duration_ms / 1000.0 + 0.3)
            consecutive = 0
            for index in range(feedback_samples):
                if self.cancel_requested:
                    raise InterruptedError("cancel requested during monitoring")
                sample = self.link.query(servo_id)
                sample["error_p"] = target_p - sample["value"]
                result["feedback"].append(sample)
                if not row["min_p"] <= sample["value"] <= row["max_p"]:
                    raise ValueError("feedback left the reviewed observation window")
                consecutive = consecutive + 1 if abs(sample["error_p"]) <= 3 else 0
                if index < feedback_samples - 1:
                    self._wait(feedback_interval_s)
            values = [sample["value"] for sample in result["feedback"]]
            result.update(
                final_p=values[-1],
                error_p=target_p - values[-1],
                observed_delta_p=values[-1] - start,
                feedback_span_p=max(values) - min(values),
                directed_feedback_change=(values[-1] - start) * (target_p - start) > 0
                and abs(values[-1] - start) > max(2, max(starts) - min(starts)),
                target_reached=consecutive >= 3,
            )
            result["observation_span_s"] = (
                result["feedback"][-1]["received_monotonic_s"]
                - result["feedback"][0]["received_monotonic_s"]
            )
            tail = values[-5:]
            result["tail_span_p"] = max(tail) - min(tail)
            result["stable_residual_observed"] = (
                result["tail_span_p"] <= 2 and not result["target_reached"]
            )
            result["status"] = (
                "TARGET_REACHED" if result["target_reached"] else "OBSERVED_NOT_AT_TARGET"
            )
        except (KeyboardInterrupt, InterruptedError) as error:
            self.used = True
            result["status"] = (
                "CANCELLED_STOP_UNVERIFIED"
                if result["motion_attempted"]
                else "CANCELLED_BEFORE_MOTION"
            )
            result["error"] = str(error) or "keyboard interrupt"
            if result["motion_attempted"]:
                result["stop_attempt"] = self.link.attempt_base_stop()
            self.link.faulted = True
        except (OSError, RuntimeError, ValueError) as error:
            self.used = True
            result["status"] = (
                "FAILED_AFTER_ATTEMPT" if result["motion_attempted"] else "PREFLIGHT_FAILED"
            )
            result["error"] = "%s: %s" % (type(error).__name__, error)
            if result["motion_attempted"]:
                result["stop_attempt"] = self.link.attempt_base_stop()
            self.link.faulted = True
        finally:
            self.link.record_best_effort("motion_result", result=result)
            result["logging_errors"] = list(self.link.log_errors)
        return result
