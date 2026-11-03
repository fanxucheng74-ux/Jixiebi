"""只读分析已结束的任务：还原检查器收到的组合P，不连接任何设备。"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import sys

from arm_project.motion_scene import SceneValidator
from arm_project.paths import PROJECT_ROOT


INPUT_NAMES = ("result.json", "events.jsonl", "input_snapshot.json", "task_plan.json")
MAX_FILE_BYTES = 32 * 1024 * 1024
MAX_EVENTS = 100000
MOTION = re.compile(r"^#([0-9]{3})P([0-9]{4})T([0-9]{4})!$")
SCENE_ERROR = "feedback rejected by the task's configured nominal scene"


def _positions(values):
    return (
        isinstance(values, list)
        and len(values) == 6
        and all(type(p) is int and 500 <= p <= 2500 for p in values)
    )


def _state_positions(state):
    if not isinstance(state, dict) or state.get("status") != "OK":
        return None
    servos = state.get("servos")
    if not isinstance(servos, list) or len(servos) != 6:
        return None
    if any(
        not isinstance(row, dict) or row.get("servo_id") != sid for sid, row in enumerate(servos)
    ):
        return None
    values = [row.get("position_p") for row in servos]
    return values if _positions(values) else None


def _commands(phase):
    targets = {}
    for command in phase.get("commands_sent", []):
        match = MOTION.fullmatch(command) if isinstance(command, str) else None
        if match is None:
            raise ValueError("phase contains an unrecognized recorded motion command")
        sid, target, _ = map(int, match.groups())
        if sid not in range(6) or not 500 <= target <= 2500 or sid in targets:
            raise ValueError("phase contains invalid or duplicate recorded targets")
        targets[sid] = target
    return targets


def _matching_reply(events, sid, value):
    # received_monotonic_s是主机接收时间，不是舵机采样时间。
    for line, event in reversed(events):
        if (
            event.get("event") == "reply"
            and event.get("field") == "position"
            and event.get("servo_id") == sid
        ):
            if event.get("value") != value:
                return None
            return {
                "events_line": line,
                "received_monotonic_s": event.get("received_monotonic_s"),
                "received_at": event.get("received_at"),
            }
    return None


def _reconstruct(phase, events, failure_error):
    """优先新拒绝事件；旧版本只在证据完整时推断端点/采样检查组合。"""
    uncertain = {"status": "UNCERTAIN", "positions6_p": None}
    rejected = [row for row in events if row[1].get("event") == "task_scene_rejected"]
    if rejected:
        line, event = rejected[-1]
        values = event.get("positions6_p")
        if event.get("phase_name") != phase.get("name") or not _positions(values):
            return {**uncertain, "reason": "拒绝事件缺少匹配阶段或完整六轴P。"}
        return {
            "status": "EXPLICIT_REJECTION_EVENT",
            "positions6_p": values,
            "events_line": line,
            "saved_scene_check": event.get("scene_check"),
            "reason": "使用运行时直接保存的拒绝组合；它仍不是硬件同步姿态。",
        }
    if SCENE_ERROR not in failure_error or phase.get("response_pass") is True:
        return {**uncertain, "reason": "记录不足以把最后一个阶段确定为场景拒绝点。"}
    starts = [i for i, (_, e) in enumerate(events) if e.get("event") == "cycle_phase_started"]
    if not starts or events[starts[-1]][1].get("name") != phase.get("name"):
        return {**uncertain, "reason": "缺少与失败阶段匹配的开始事件。"}
    phase_events = events[starts[-1] + 1 :]
    states = [(line, e) for line, e in phase_events if e.get("event") == "arm_state"]
    baseline = _state_positions(states[-1][1].get("state")) if states else phase.get("starts_p")
    if not _positions(baseline):
        return {**uncertain, "reason": "缺少有效六轴基线，不能用历史目标补造读数。"}
    active = phase.get("end_active", [])
    targets = _commands(phase)
    if active:
        # 父执行器在所有活动轴端点完成后才做最终几何检查；不能只拿最后arm_state。
        # 缺轴/容差未通过时可能停在更早分支，不能把部分端点冒充最终guard输入。
        if (
            not isinstance(active, list)
            or len(active) != len(targets)
            or any(not isinstance(row, dict) for row in active)
            or {row.get("servo_id") for row in active} != set(targets)
            or any(
                row.get("target_p") != targets[row["servo_id"]]
                or row.get("in_handoff_corridor") is not True
                or not any(
                    row.get(key) is True
                    for key in (
                        "direction_response",
                        "already_near_target",
                        "unchanged_ordered_target_hold",
                    )
                )
                for row in active
            )
        ):
            return {**uncertain, "reason": "活动轴端点不完整或交接检查未通过。"}
        values = list(baseline)
        replies = []
        last_state_line = states[-1][0] if states else events[starts[-1]][0]
        endpoint_events = [row for row in phase_events if row[0] > last_state_line]
        for row in active:
            sid, value = row["servo_id"], row.get("feedback_p")
            values[sid] = value
            reply = _matching_reply(endpoint_events, sid, value)
            if reply is None:
                return {**uncertain, "reason": "端点与逐轴reply不一致或缺少对应回复。"}
            replies.append({"servo_id": sid, **reply})
        if not _positions(values):
            return {**uncertain, "reason": "端点包含非法P，不能重算场景。"}
        return {
            "status": "RECONSTRUCTED_ENDPOINT_GUARD",
            "positions6_p": values,
            "baseline_events_line": states[-1][0] if states else None,
            "active_replies": replies,
            "reason": "以最后六轴基线叠加完整活动轴端点；保留非活动轴旧时刻读数。",
        }
    if states:
        line, event = states[-1]
        # 已记录cycle_sample的状态在guard中通过过，不应再当作失败状态。
        if not any(n > line and e.get("event") == "cycle_sample" for n, e in phase_events):
            return {
                "status": "RECONSTRUCTED_SAMPLE_GUARD",
                "positions6_p": baseline,
                "events_line": line,
                "reason": "最后完整arm_state未成为成功cycle_sample；按旧执行顺序还原。",
            }
    return {**uncertain, "reason": "最后保存的状态已通过采样检查，不能认作拒绝姿态。"}


def _span_ms(replies):
    values = [row.get("received_monotonic_s") for row in replies]
    if not values or any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
        return None
    return (max(values) - min(values)) * 1000


def inspect_failure(run_dir):
    """返回有来源哈希的纯离线报告，不修改输入目录。"""
    run_dir = Path(run_dir).resolve(strict=True)
    raw, hashes = {}, {}
    for name in INPUT_NAMES:
        path = run_dir / name
        if not path.is_file() or path.stat().st_size > MAX_FILE_BYTES:
            raise ValueError(f"missing or oversized input: {name}")
        raw[name] = path.read_bytes()
        hashes[name] = hashlib.sha256(raw[name]).hexdigest()
    events = []
    for line, value in enumerate(raw["events.jsonl"].decode("utf-8").splitlines(), start=1):
        if not value.strip():
            continue
        event = json.loads(value)
        if not isinstance(event, dict):
            raise ValueError("event must be a JSON object")
        events.append((line, event))
        if len(events) > MAX_EVENTS:
            raise ValueError("event count exceeds inspection budget")
    result = json.loads(raw["result.json"])
    snapshot = json.loads(raw["input_snapshot.json"])
    task = json.loads(raw["task_plan.json"])
    if any(not isinstance(value, dict) for value in (result, snapshot, task)):
        raise ValueError("result, snapshot and task must be JSON objects")
    scene = SceneValidator(snapshot["model"], snapshot["scene_config"])
    sequence = result.get("sequence") or {}
    phases = sequence.get("phases") or []
    phase = phases[-1] if phases else {}
    reconstruction = _reconstruct(phase, events, str(result.get("error") or ""))
    actual = reconstruction["positions6_p"]
    target = list(actual) if actual is not None else None
    active_errors = []
    if target is not None:
        for sid, held in phase.get("continuing_goals", {}).items():
            target[int(sid)] = held["target_p"]
        for sid, value in _commands(phase).items():
            target[sid] = value
            active_errors.append(
                {
                    "servo_id": sid,
                    "target_p": value,
                    "feedback_p": actual[sid],
                    "feedback_minus_target_p": actual[sid] - value,
                }
            )
    actual_scene = scene.inspect(actual[:4]) if actual else None
    target_scene = scene.inspect(target[:4]) if target else None
    findings = []
    if reconstruction["status"] == "RECONSTRUCTED_ENDPOINT_GUARD":
        findings.append("拒绝点在活动轴端点回读完成后的检查；最后成功运动中样本不是失败姿态。")
    if actual_scene and target_scene and target_scene["valid"] and not actual_scene["valid"]:
        findings.append("理想目标通过场景检查，但回读组合未通过；逐轴容差通过不等于组合净空通过。")
    findings.append("毫米值来自当次未标定模型；组合P不证明硬件同步姿态、实物碰撞或物理停止。")
    report = {
        "schema": "jixiebi.transfer_failure_inspection.v1",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "hardware_io": False,
        "motion_authorized": False,
        "source_run_dir": str(run_dir),
        "source_hashes_sha256": hashes,
        "task_id": task.get("task_id"),
        "recorded_source": result.get("source"),
        "recorded_status": result.get("status"),
        "recorded_error": result.get("error"),
        "phase_name": phase.get("name"),
        "commands_sent": phase.get("commands_sent", []),
        "starts6_p": phase.get("starts_p"),
        "reconstruction": reconstruction,
        "active_axis_errors_p": active_errors,
        "active_reply_host_receive_span_ms": _span_ms(reconstruction.get("active_replies", [])),
        "recomputed_static_scene": actual_scene,
        "target_positions6_p": target,
        "target_static_scene": target_scene,
        "physical_geometry_verified": False,
        "simultaneous": False,
        "findings": findings,
    }
    # 分析期间输入若被别的进程改变就拒绝交付，避免哈希与实际读取版本不一致。
    if any(hashlib.sha256((run_dir / n).read_bytes()).hexdigest() != h for n, h in hashes.items()):
        raise RuntimeError("source records changed during inspection")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        run_dir = args.run_dir if args.run_dir.is_absolute() else PROJECT_ROOT / args.run_dir
        output = args.output if args.output.is_absolute() else PROJECT_ROOT / args.output
        output = output.resolve()
        if output.is_relative_to(run_dir.resolve()):
            raise ValueError("output must be outside the original run directory")
        if output.exists():
            raise FileExistsError("output already exists; records will not be overwritten")
        report = inspect_failure(run_dir)
        output.mkdir(parents=True, exist_ok=False)
        with (output / "report.json").open("x", encoding="utf-8") as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
        print("离线还原:", report["reconstruction"]["status"])
        print("阶段:", report["phase_name"])
        print("组合六轴P:", report["reconstruction"]["positions6_p"])
        check = report["recomputed_static_scene"]
        if check:
            print(
                "名义场景:", check.get("code"), "净空mm:", check.get("minimum_table_clearance_mm")
            )
        print("REPORT:", output / "report.json")
        return 0
    except (OSError, ValueError, KeyError, RuntimeError) as error:
        print("ERROR:", type(error).__name__ + ":", error, file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
