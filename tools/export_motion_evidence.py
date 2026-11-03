"""Export immutable hardware evidence; never connects to a device.

This is an evaluation/evidence supplement, not instruction-to-code SFT data.
Position feedback remains protocol P with host receive timestamps. A sequence
result and an independent physical observation are deliberately separate.
"""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re


DEFAULT_TRIAL_IDS = (
    "paper_empty_pick_place_timed_20261101_05",
    "paper_empty_pick_place_timed_20261101_06",
)
RAW_FILES = ("result.json", "spec.json", "events.jsonl", "task_attempt.json")
TELEMETRY_FIELDS = (
    "trial_id",
    "source_event_index",
    "phase_name",
    "servo_id",
    "position_p",
    "received_at",
    "received_monotonic_s",
    "timestamp_origin",
    "position_unit",
    "simultaneous",
)
OBSERVATIONS = {
    DEFAULT_TRIAL_IDS[0]: {
        "source": "USER_ONSITE_REPORT",
        "physical_cycle_confirmed": True,
        "text": "完整完成，纸团已放回，末态竖直空爪",
        "contact_with_table_reported": None,
        "persistent_noise_reported": None,
        "object_still_held_at_stop": False,
    },
    DEFAULT_TRIAL_IDS[1]: {
        "source": "USER_ONSITE_REPORT",
        "physical_cycle_confirmed": None,
        "text": "没有，先停止这次测试吧，按照这个要求，整理数据集",
        "contact_with_table_reported": False,
        "persistent_noise_reported": False,
        "object_still_held_at_stop": None,
        "interpretation": "没有回答对应碰桌/持续嗡嗡；未明确回答纸团是否仍夹着。",
    },
}


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def load_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def require(condition, message):
    if not condition:
        raise ValueError(message)


def _read_trial(root, trial_id):
    require(
        isinstance(trial_id, str) and re.fullmatch(r"[A-Za-z0-9_-]+", trial_id),
        "unsafe trial_id",
    )
    folder = root / "runs" / "timed_cycles" / trial_id
    source_hashes = {
        str((folder / name).relative_to(root)).replace("\\", "/"): digest(folder / name)
        for name in RAW_FILES
    }
    result, spec, budget = (
        load_json(folder / name) for name in ("result.json", "spec.json", "task_attempt.json")
    )
    events = [
        json.loads(line)
        for line in (folder / "events.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    require(isinstance(result, dict) and result.get("source") == "HARDWARE", "not hardware")
    sequence = result.get("sequence")
    require(isinstance(sequence, dict), "missing sequence")
    require(isinstance(spec, dict) and spec.get("review_id") == trial_id, "review_id mismatch")
    require(isinstance(spec.get("phases"), list), "missing planned phases")
    require(isinstance(sequence.get("phases"), list), "missing executed phases")
    require(
        sequence.get("status") in ("RESPONSE_SEQUENCE_COMPLETED", "FAILED_AFTER_ATTEMPT"),
        "unsupported sequence status",
    )
    require(type(sequence.get("response_pass")) is bool, "missing response_pass")
    require(type(sequence.get("elapsed_s")) in (int, float), "missing elapsed_s")
    require(
        math.isfinite(sequence["elapsed_s"]) and sequence["elapsed_s"] >= 0,
        "invalid elapsed_s",
    )
    require(type(sequence.get("within_deadline")) is bool, "missing within_deadline")
    require(type(spec.get("deadline_ms")) is int, "missing deadline_ms")
    require(all(isinstance(e, dict) and "event" in e for e in events), "invalid events")
    expected = [
        f"#{t['servo_id']:03d}P{t['target_p']}T{p['duration_ms']}!"
        for p in spec["phases"]
        for t in p["targets"]
    ]
    sent = [e["command"] for e in events if e["event"] == "motion_tx"]
    require(sent == expected[: len(sent)] and len(sent) <= len(expected), "motion prefix mismatch")
    require(
        isinstance(budget, dict)
        and isinstance(budget.get("plan"), dict)
        and budget["plan"].get("commands")
        == [
            [f"#{t['servo_id']:03d}P{t['target_p']}T{p['duration_ms']}!" for t in p["targets"]]
            for p in spec["phases"]
        ],
        "reserved command budget mismatch",
    )
    completed = sequence["status"] == "RESPONSE_SEQUENCE_COMPLETED"
    require(completed or not sequence["response_pass"], "failed trial claims response_pass")
    require(
        not completed
        or (
            sequence["response_pass"]
            and sequence["within_deadline"]
            and sequence["elapsed_s"] <= spec["deadline_ms"] / 1000
            and len(sent) == len(expected)
            and len(sequence["phases"]) == len(spec["phases"])
            and all(p.get("response_pass") is True for p in sequence["phases"])
        ),
        "inconsistent completed trial",
    )
    observation = OBSERVATIONS.get(trial_id)
    trial = {
        "schema_version": 1,
        "trial_id": trial_id,
        "source": "HARDWARE",
        "status": sequence["status"],
        "sequence_completed": completed,
        "response_pass": sequence["response_pass"],
        "elapsed_s": sequence["elapsed_s"],
        "deadline_ms": spec["deadline_ms"],
        "within_deadline": sequence["within_deadline"],
        "error": sequence.get("error") or result.get("error"),
        "motion_frames_sent": len(sent),
        "planned_motion_frames": len(expected),
        "sent_frames_exact_plan_prefix": True,
        "raw_physical_task_verified": sequence.get("physical_task_verified"),
        "raw_physical_stop_verified": sequence.get("physical_stop_verified"),
        "raw_strict_position_verified": sequence.get("strict_position_verified"),
        "external_observation": observation,
        "stop_attempts": sequence.get("stops", []),
        "transport_closed": result.get("transport_closed"),
        "logging_errors": result.get("logging_errors"),
        "video_path": None,
        "video_status": "excluded_by_user",
        "video_alignment_verified": False,
        "source_sha256": source_hashes,
    }
    phases = []
    for index, phase in enumerate(sequence["phases"]):
        require(
            index < len(spec["phases"]) and phase.get("name") == spec["phases"][index]["name"],
            "phase order mismatch",
        )
        phases.append(
            {
                "schema_version": 1,
                "trial_id": trial_id,
                "phase_index": index,
                "phase_name": phase["name"],
                "planned_duration_ms": spec["phases"][index]["duration_ms"],
                "planned_dwell_ms": spec["phases"][index].get("dwell_ms", 0),
                "targets": spec["phases"][index]["targets"],
                "commands_sent": phase.get("commands_sent", []),
                "start_p6": phase.get("starts_p"),
                "end_active": phase.get("end_active", []),
                "continuing_goals": phase.get("continuing_goals", {}),
                "response_pass": phase.get("response_pass"),
                "physical_phase_success_verified": None,
                "trial_status": sequence["status"],
            }
        )
    telemetry = []
    current_phase = ""
    for index, event in enumerate(events):
        if event["event"] == "cycle_phase_started":
            current_phase = event["name"]
        elif event["event"] == "cycle_phase_end":
            current_phase = ""
        elif event["event"] == "reply" and event.get("field") == "position":
            require(type(event.get("servo_id")) is int, "invalid servo_id")
            require(0 <= event["servo_id"] < 6, "invalid servo_id")
            require(type(event.get("value")) is int, "invalid position")
            require(isinstance(event.get("received_at"), str), "missing receive timestamp")
            require(
                type(event.get("received_monotonic_s")) in (int, float)
                and math.isfinite(event["received_monotonic_s"]),
                "missing monotonic timestamp",
            )
            telemetry.append(
                {
                    "trial_id": trial_id,
                    "source_event_index": index,
                    "phase_name": current_phase,
                    "servo_id": event["servo_id"],
                    "position_p": event["value"],
                    "received_at": event["received_at"],
                    "received_monotonic_s": event["received_monotonic_s"],
                    "timestamp_origin": "host_receive",
                    "position_unit": "protocol_P",
                    "simultaneous": False,
                }
            )
    require(bool(telemetry), "no position feedback")
    return trial, phases, telemetry, source_hashes


def export_motion_evidence(project_root, output, trial_ids=DEFAULT_TRIAL_IDS):
    """Create one new export directory, preserving all raw input bytes."""
    root, output = Path(project_root).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError(output)
    require(output != root and root / "runs" not in output.parents, "output overlaps raw runs")
    require(len(trial_ids) > 0 and len(set(trial_ids)) == len(trial_ids), "duplicate/empty trials")
    trials, phases, telemetry, source_hashes = [], [], [], {}
    for trial_id in trial_ids:
        trial, trial_phases, trial_telemetry, hashes = _read_trial(root, trial_id)
        trials.append(trial)
        phases.extend(trial_phases)
        telemetry.extend(trial_telemetry)
        source_hashes.update(hashes)
    output.mkdir(parents=True, exist_ok=False)
    for filename, rows in (("trials.jsonl", trials), ("phase_events.jsonl", phases)):
        with (output / filename).open("x", encoding="utf-8", newline="\n") as stream:
            for row in rows:
                stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
    with (output / "telemetry.csv").open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=TELEMETRY_FIELDS)
        writer.writeheader()
        writer.writerows(telemetry)
    require(
        all(digest(root / path) == value for path, value in source_hashes.items()),
        "raw source changed during export",
    )
    provenance = {
        "schema_version": 1,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "purpose": "evaluation_evidence_not_sft_targets",
        "trial_count": len(trials),
        "phase_count": len(phases),
        "position_reply_count": len(telemetry),
        "source_sha256": source_hashes,
        "output_sha256": {
            name: digest(output / name)
            for name in ("trials.jsonl", "phase_events.jsonl", "telemetry.csv")
        },
        "raw_files_unchanged": True,
        "hardware_opened": False,
        "motion_sent": False,
        "measured_xyz_available": False,
        "force_available": False,
        "simultaneous_joint_state_available": False,
        "video": {
            "status": "excluded_by_user",
            "trial_binding_verified": False,
            "note": "用户明确本次不用管视频；不查找、复制、标注或绑定录像。",
        },
    }
    with (output / "provenance.json").open("x", encoding="utf-8") as stream:
        json.dump(provenance, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    return provenance


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    report = export_motion_evidence(args.root, args.output)
    print(
        f"EVIDENCE: {report['trial_count']} trials, {report['phase_count']} phases, "
        f"{report['position_reply_count']} sequential position replies"
    )
    print("OUTPUT:", args.output.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
