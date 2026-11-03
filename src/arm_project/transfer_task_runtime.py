"""一次请求完成入口与全部搬运阶段；默认不连接设备，不把名义方案当实物许可。"""

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
from threading import Event, Lock
import time
from uuid import UUID, uuid4

from arm_project.arm_link import SerialTransport, ServoLink
from arm_project.arm_state import ArmStateReader
from arm_project.clearance_recovery import task_scene_validator
from arm_project.fixed_transfer import plan_fixed_transfer
from arm_project.general_transfer_entry import (
    plan_general_transfer_entry,
    plan_simulated_general_entry,
)
from arm_project.handoff_envelope import HandoffEnvelopeError, require_handoff_envelopes
from arm_project.motion_scene import SceneValidator
from arm_project.paths import PROJECT_ROOT
from arm_project.servo_protocol import validate_read_query
from arm_project.transfer_entry import plan_simulated_transfer_entry, plan_transfer_entry
from arm_project.transfer_task_motion import (
    TaskTransport,
    TransferTaskController,
    compile_motion_phases,
    motion_phases_sha256,
    validate_task_review,
)
from arm_project.transfer_task_plan import compose_transfer_task


def unique_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("JSON字段重复: " + key)
        result[key] = value
    return result


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"), object_pairs_hook=unique_pairs)


def config_path(value):
    if not isinstance(value, str) or not value.strip():
        raise ValueError("配置文件路径必须明确")
    path = Path(value).expanduser()
    return path if path.is_absolute() else PROJECT_ROOT / path


def load_settings(config):
    required = {
        "schema",
        "task_id",
        "model_path",
        "request_path",
        "policy_path",
        "gripper",
        "reviews",
    }
    optional = {"entry_planner_path", "scene_path", "task_policy_path"}
    if not isinstance(config, dict) or required - set(config) or set(config) - required - optional:
        raise ValueError("完整任务配置字段不匹配")
    if bool(config.get("entry_planner_path")) != bool(config.get("scene_path")):
        raise ValueError("通用入口须同时提供场景与规划参数")
    if config.get("task_policy_path") and not config.get("entry_planner_path"):
        raise ValueError("算法路径策略只用于通用入口")
    if config["schema"] != "jixiebi.transfer_task_config.v1":
        raise ValueError("完整任务配置版本不匹配")
    if not isinstance(config["task_id"], str) or not re.fullmatch(
        r"[A-Za-z0-9_-]{1,80}", config["task_id"]
    ):
        raise ValueError("task_id需为有限长度的字母、数字、下划线或连字符")
    if not isinstance(config["reviews"], dict) or set(config["reviews"]) != {
        "ALREADY_AT_START",
        "NOMINAL_ENTRY_PROPOSAL",
    }:
        raise ValueError("必须显式列出左高位与右高位的任务审阅路径")
    model, request, policy = [
        read_json(config_path(config[key])) for key in ("model_path", "request_path", "policy_path")
    ]
    transfer = plan_fixed_transfer(model, request)
    if request["table_z_mm"] != 0:
        raise ValueError("当前完整任务执行器只支持显式table_z_mm=0，不能混用桌面坐标")
    return model, transfer, policy


def demo_task(config, posture):
    """一条命令生成整个任务，不打开串口、相机或消费实物预算。"""
    model, transfer, policy = load_settings(config)
    if posture not in {"left_hover", "right_hover", "upright"}:
        raise ValueError("未知演示姿态")
    if posture == "upright":
        positions = [1500] * 4
    else:
        key = "place_hover" if posture == "right_hover" else "pick_hover"
        positions = list(transfer["nominal_endpoints_p"][key])
    if config.get("entry_planner_path"):
        entry = plan_simulated_general_entry(
            model,
            transfer,
            positions + [1500, 1200],
            policy,
            read_json(config_path(config["scene_path"])),
            read_json(config_path(config["entry_planner_path"])),
        )
    else:
        entry = plan_simulated_transfer_entry(model, transfer, positions + [1500, 1200], policy)
    return compose_transfer_task(transfer, entry, config["gripper"], config["task_id"])


def save_new(path, value):
    # 排他创建并落盘。调用者必须在本函数成功返回以后才能发送预算内定位帧。
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.flush()
        os.fsync(stream.fileno())


def review_draft(task, model):
    """导出未批准的精确清单供测量/审阅；不会自动扩大范围或批准实验。"""
    phases = compile_motion_phases(task)
    return {
        "schema": "jixiebi.reviewed_transfer_task.v1",
        "review_id": task["task_id"],
        "whole_path_reviewed_for_motion": False,
        "evidence": "待填写本机路径、行程、空爪及工作区审阅依据；本文件尚未获准执行",
        "start_intervals_p": None,
        "observation_bounds_p": None,
        "geometry_mm": model["geometry_mm"],
        "table_z_mm": 0,
        "deadline_ms": None,
        "motion_phases_sha256": motion_phases_sha256(task),
        "phases": [
            {
                "name": p["name"],
                "duration_ms": p["duration_ms"],
                "dwell_ms": 0,
                "targets": [
                    {"servo_id": int(sid), "target_p": target, "end_tolerance_p": None}
                    for sid, target in p["targets"].items()
                ],
            }
            for p in phases
        ],
    }


class _TaskSerial(SerialTransport):
    """只由下面的查询/精确阶段门禁调用；不改动默认串口驱动的旧白名单。"""

    def write(self, command):
        return self.port.write(command)


class _QueryStage:
    """运行初始只允许查询；选出并核对整趟清单后换成TaskTransport，共用同一串口。"""

    def __init__(self, raw):
        self.raw = raw
        self.source = raw.source

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
        validate_read_query(command)
        return self.raw.write(command)


class TransferTaskRunner:
    """一个按钮的一次性任务；串口由run独占，cancel只设标志、不并发写串口。"""

    def __init__(self, config, port, progress=None):
        self.config = config
        self.port = port
        self.progress = progress
        self.cancel_event = Event()
        self.lock = Lock()
        self.used = self.finished = False
        self.controller = None
        self.progress_errors = []

    def cancel(self):
        with self.lock:
            if self.finished:
                return {"status": "FINISHED", "physical_stop_verified": False}
            self.cancel_event.set()
            if self.controller is not None:
                self.controller.cancel()
        return {"status": "CANCEL_REQUESTED", "physical_stop_verified": False}

    def _check_cancel(self):
        if self.cancel_event.is_set():
            raise RuntimeError("完整任务已取消；不自动回正或重新下发")

    def _notify(self, status, **values):
        if self.progress is not None:
            try:
                self.progress({"status": status, **values})
            except Exception as error:
                # 状态反馈中断时请求控制线程取消；不让finally中的发布异常掩盖原始记录。
                self.progress_errors.append(type(error).__name__ + ": " + str(error))
                self.cancel()

    def run(self):
        with self.lock:
            if self.used:
                return {"status": "ALREADY_USED", "response_pass": False}
            self.used = True
        result = {
            "status": "FAILED",
            "response_pass": False,
            "physical_task_verified": False,
            "physical_stop_verified": False,
            "error": None,
            "sequence": None,
        }
        wall_started = time.monotonic()
        folder = None
        raw = link = None
        phase_times = []
        try:
            self._check_cancel()
            model, transfer, policy = load_settings(self.config)
            if not isinstance(self.port, str) or not self.port:
                raise ValueError("实物任务必须显式给出串口")
            # 跨代码快照共用原项目预算；创建目录即消耗本次调用，不能换快照重放。
            root = Path.home() / "Jixiebi"
            if not (root / ".arm-project-root").is_file():
                raise ValueError("缺少~/Jixiebi原项目标记，无法共用任务预算")
            candidate = root / "runs/transfer_tasks" / self.config["task_id"]
            candidate.mkdir(parents=True, exist_ok=False)
            # 只有成功独占新目录后才允许finally落盘，不能写入先前失败/已消费任务目录。
            folder = candidate
            save_new(folder / "config.json", self.config)
            result["result_path"] = str(folder / "result.json")
            boot = str(UUID(Path("/proc/sys/kernel/random/boot_id").read_text().strip()))
            with (folder / "events.jsonl").open("x", encoding="utf-8") as log:

                def record(event, **values):
                    log.write(json.dumps({"event": event, **values}, ensure_ascii=False) + "\n")
                    log.flush()
                    if event == "cycle_phase_started":
                        phase_times.append({"name": values["name"], "start_s": values["elapsed_s"]})
                        self._notify("RUNNING", phase=values["name"], elapsed_s=values["elapsed_s"])

                raw = _TaskSerial(self.port)
                with ServoLink(_QueryStage(raw), record=record) as link:
                    result["source"] = raw.source
                    self._notify("READING_CURRENT_STATE")
                    self._check_cancel()
                    state = ArmStateReader(link).read_state()
                    state["host_boot_id"] = boot
                    entry_function = plan_transfer_entry
                    entry_options = {}
                    if self.config.get("entry_planner_path"):
                        entry_function = plan_general_transfer_entry
                        entry_options = {
                            "scene_config": read_json(config_path(self.config["scene_path"])),
                            "settings": read_json(config_path(self.config["entry_planner_path"])),
                        }
                    entry = entry_function(
                        model,
                        transfer,
                        {
                            "schema": "jixiebi.raw_state.v1",
                            "session_id": uuid4().hex,
                            "batch_index": 1,
                            "state": state,
                        },
                        policy,
                        now_monotonic_s=link.clock.monotonic(),
                        host_boot_id=boot,
                        **entry_options,
                    )
                    # 失败也保留原始入口判断和使用的参数，后续无需猜是哪一段余量不足。
                    save_new(folder / "entry_plan.json", entry)
                    save_new(
                        folder / "input_snapshot.json",
                        {
                            "model": model,
                            "request": transfer["input_assumptions"],
                            "entry_policy": policy,
                            **entry_options,
                        },
                    )
                    task = compose_transfer_task(
                        transfer, entry, self.config["gripper"], self.config["task_id"]
                    )
                    save_new(folder / "task_plan.json", task)
                    if task["status"] == "BLOCKED":
                        result.update(status="ENTRY_BLOCKED", error=entry["reason"])
                    else:
                        self._check_cancel()
                        review_path = self.config["reviews"][entry["status"]]
                        review = None
                        if self.config.get("task_policy_path"):
                            from arm_project.transfer_task_policy import build_policy_review

                            policy_review = read_json(config_path(self.config["task_policy_path"]))
                            if policy_review.get("enabled") is True:
                                review = build_policy_review(
                                    task,
                                    model,
                                    entry_options["scene_config"],
                                    entry_options["settings"],
                                    policy_review,
                                )
                        elif review_path is not None:
                            review = read_json(config_path(review_path))
                        if review is None:
                            save_new(folder / "review_draft.json", review_draft(task, model))
                            result.update(
                                status="REVIEW_REQUIRED", error="当前入口分支尚无完整任务审阅"
                            )
                        else:
                            spec = validate_task_review(task, review)
                            if spec.review_id != self.config["task_id"]:
                                raise ValueError("配置task_id必须与本次review_id一致")
                            if self.config.get("entry_planner_path"):
                                # 静态外部审阅同样不能跳过误差组合检查。只读当前状态后、
                                # 尚未建立TaskTransport或消费运动预算时保存完整报告。
                                handoff_started = time.monotonic()

                                def handoff_guard():
                                    self._check_cancel()
                                    if time.monotonic() - handoff_started >= 3.0:
                                        raise ValueError(
                                            "handoff preflight exceeded 3-second budget"
                                        )

                                handoff = require_handoff_envelopes(
                                    spec,
                                    SceneValidator(model, entry_options["scene_config"]),
                                    check_budget=handoff_guard,
                                )
                                save_new(folder / "handoff_envelope.json", handoff)
                            save_new(folder / "review.json", spec.to_dict())
                            gate = TaskTransport(self.port, spec, raw=raw)
                            # 只替换门禁，不关闭/重开串口。此后全部定位帧受精确清单限制。
                            link.transport = gate

                            def reserve(plan):
                                self._check_cancel()
                                save_new(folder / "motion_attempt.json", {"plan": plan})

                            self.controller = TransferTaskController(
                                link,
                                spec,
                                reserve,
                                cancel_file=folder / "cancel.request",
                                **(
                                    {
                                        "state_validator": task_scene_validator(
                                            SceneValidator(model, entry_options["scene_config"]),
                                            task,
                                        )
                                    }
                                    if self.config.get("entry_planner_path")
                                    else {}
                                ),
                            )
                            if self.cancel_event.is_set():
                                self.controller.cancel()
                            sequence = self.controller.run()
                            result.update(
                                status=sequence["status"],
                                response_pass=sequence["response_pass"],
                                sequence=sequence,
                                error=sequence["error"],
                            )
                            result["phase_start_times"] = phase_times
                            # 按连续任务中的边界计每轮，不把复位拆成另一次用户调用。
                            # 最后一轮包含结束六轴读取；失败时只保留已开始阶段，不制造成功轮次。
                            starts = {item["name"]: item["start_s"] for item in phase_times}
                            result["cycle_elapsed_s"] = []
                            ended = {
                                item["name"]
                                for item in sequence["phases"]
                                if item.get("response_pass") is True
                            }
                            for cycle in range(1, task["cycle_count"] + 1):
                                name = f"cycle_{cycle:02d}_descend_left"
                                finish = f"cycle_{cycle:02d}_rotate_left"
                                next_name = f"cycle_{cycle + 1:02d}_descend_left"
                                if name in starts and finish in ended:
                                    stop = starts.get(next_name, sequence.get("elapsed_s"))
                                    if stop is not None:
                                        result["cycle_elapsed_s"].append(
                                            {
                                                "cycle_index": cycle,
                                                "elapsed_s": stop - starts[name],
                                                "drive_sequence_completed": True,
                                                "physical_grasp_verified": False,
                                            }
                                        )
                result["transport_closed"] = raw.closed
                result["logging_errors"] = list(link.log_errors)
                if not raw.closed or link.log_errors:
                    result.update(status="LOGGING_OR_CLOSE_FAILED", response_pass=False)
        except HandoffEnvelopeError as error:
            result.update(status="HANDOFF_BLOCKED", response_pass=False, error=str(error))
            # 策略生成和外部审阅两条入口都保留同样的拒绝报告；此分支没有运动预算。
            if folder is not None:
                try:
                    save_new(folder / "handoff_envelope.json", error.report)
                except OSError as log_error:
                    result.update(
                        status="LOGGING_FAILED",
                        error=f"{error}; envelope report logging failed: {log_error}",
                    )
        except (Exception, KeyboardInterrupt) as error:
            result.update(
                status="CANCELLED" if self.cancel_event.is_set() else "FAILED",
                response_pass=False,
                error=type(error).__name__ + ": " + str(error),
            )
        finally:
            with self.lock:
                self.finished = True
            result["transport_closed"] = raw is None or raw.closed
            result["logging_errors"] = list(link.log_errors) if link is not None else []
            result["task_wall_elapsed_s"] = time.monotonic() - wall_started
            # 先发布结束，再固化返回记录，保证最终发布失败也进入result而不误报成功。
            self._notify(
                result["status"],
                response_pass=result["response_pass"],
                result_path=result.get("result_path"),
                error=result["error"],
            )
            result["progress_errors"] = list(self.progress_errors)
            if (
                result["progress_errors"]
                or result["logging_errors"]
                or not result["transport_closed"]
            ):
                result.update(status="LOGGING_OR_CLOSE_FAILED", response_pass=False)
            if folder is not None:
                try:
                    save_new(folder / "result.json", result)
                except OSError as error:
                    result.update(status="LOGGING_FAILED", response_pass=False, error=str(error))
        return result


def save_demo(task, output=None):
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    folder = output or PROJECT_ROOT / "runs" / ("complete_task_" + stamp)
    folder = Path(folder)
    if not folder.is_absolute():
        folder = PROJECT_ROOT / folder
    folder.mkdir(parents=True, exist_ok=False)
    save_new(folder / "task_plan.json", task)
    return folder / "task_plan.json"
