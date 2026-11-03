"""Offline evidence preparation. Never imports hardware or authorizes motion.

Inventory supports completed, error-free unified-driver sessions. Hashes and
cross-checks establish file consistency, not authenticity of physical claims.
Measurement schema v1: kind joint_angle/gripper_width, p_kind observed_feedback,
measurement_method/reference nonempty, and samples containing the existing
joint_calibration fields plus observed_at, feedback_evidence and
measurement_evidence. Each evidence is {path, sha256}; feedback_evidence also
has a 1-based line selecting a raw-backed position reply in events.jsonl.
External measurements and feedback must share observed_at. File paths are
project-relative and cannot escape the supplied root. Empty templates cannot fit.
"""

from collections import Counter
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import re

from arm_project.joint_calibration import (
    build_gripper_table,
    fit_angle_mapping,
    summarize_directional_residuals,
)


class EvidenceError(ValueError):
    """An evidence file is incomplete, inconsistent or unsupported."""


def _need(condition, message):
    if not condition:
        raise EvidenceError(message)


def _integer(value, low, high, label):
    _need(type(value) is int and low <= value <= high, label + " is not a valid integer")
    return value


def _text(value, label):
    _need(type(value) is str and bool(value.strip()), label + " is required")
    return value


def _time(value):
    try:
        stamp = datetime.fromisoformat(_text(value, "timestamp"))
    except ValueError as error:
        raise EvidenceError("invalid timestamp") from error
    _need(stamp.utcoffset() is not None, "timestamp needs a timezone")
    return stamp


def _path(root, value):
    root = Path(root).resolve()
    path = Path(value)
    path = (path if path.is_absolute() else root / path).resolve()
    _need(path.is_relative_to(root), "evidence path escapes project root")
    return path


def _load(path):
    def bad_constant(value):
        raise EvidenceError("non-finite JSON constant: " + value)

    def unique_pairs(pairs):
        data = {}
        for key, value in pairs:
            _need(key not in data, "duplicate JSON key: " + key)
            data[key] = value
        return data

    try:
        value = json.loads(
            path.read_text(encoding="utf-8-sig"),
            parse_constant=bad_constant,
            object_pairs_hook=unique_pairs,
        )
        _need(type(value) is dict, "JSON document must be an object")
        return value
    except (OSError, json.JSONDecodeError) as error:
        raise EvidenceError(str(error)) from error


def _events(path):
    try:
        lines = path.read_text(encoding="utf-8-sig").splitlines()
        values = [json.loads(line) for line in lines]
    except (OSError, json.JSONDecodeError) as error:
        raise EvidenceError(str(error)) from error
    _need(values and all(type(row) is dict for row in values), "empty/invalid event log")
    return values


def file_evidence(root, path):
    root = Path(root).resolve()
    path = _path(root, path)
    try:
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
    except OSError as error:
        raise EvidenceError(str(error)) from error
    return {"path": path.relative_to(root).as_posix(), "sha256": digest}


def _checked_evidence(root, evidence):
    _need(type(evidence) is dict, "evidence object required")
    _need(
        type(evidence.get("path")) is str and not Path(evidence["path"]).is_absolute(),
        "evidence path must be relative",
    )
    actual = file_evidence(root, evidence["path"])
    _need(evidence.get("sha256") == actual["sha256"], "evidence SHA256 mismatch")
    return _path(root, evidence["path"])


def _raw_replies(events):
    """Match each serial query, complete write, RX bytes and parsed reply in order."""
    query = None
    pending_write = None
    raw = bytearray()
    replies = []
    motions = []
    opened = [e for e in events if e.get("event") == "opened"]
    _need(len(opened) == 1 and opened[0].get("source") == "HARDWARE", "not a hardware session")
    for line, event in enumerate(events, 1):
        kind = event.get("event")
        if kind == "wire_tx":
            _need(pending_write is None and query is None, "overlapping or unanswered command")
            command = _text(event.get("command"), "wire command")
            _need(event.get("tx_hex") == command.encode("ascii").hex(), "TX ASCII/hex mismatch")
            pending_write = event
            if event.get("kind") == "query":
                match = re.fullmatch(r"#(00[0-5])(PRAD|PVER|PMOD)!", command)
                _need(match is not None, "unsupported query")
                query = (int(match[1]), match[2])
                raw.clear()
            else:
                _need(
                    event.get("kind") == "motion"
                    and re.fullmatch(r"#00[0-5]P\d{4}T\d{4}!", command),
                    "unexpected stop/config/broadcast or malformed motion",
                )
                motions.append(event)
        elif kind == "wire_write_returned":
            _need(pending_write is not None, "write result without wire TX")
            for key in ("kind", "command", "tx_hex"):
                _need(event.get(key) == pending_write.get(key), "write result differs from TX")
            size = len(pending_write["command"].encode("ascii"))
            _need(
                type(event.get("bytes_written")) is int
                and event["bytes_written"] == size
                and event.get("expected_bytes") == size,
                "short/incomplete write",
            )
            pending_write = None
        elif kind == "rx":
            _need(query is not None and pending_write is None, "unsolicited RX")
            try:
                raw.extend(bytes.fromhex(event["rx_hex"]))
            except (KeyError, ValueError, TypeError) as error:
                raise EvidenceError("invalid raw RX") from error
        elif kind == "reply":
            _need(query is not None and pending_write is None, "reply without complete query")
            axis, operation = query
            field = {"PRAD": "position", "PVER": "version", "PMOD": "mode"}[operation]
            _need(
                event.get("servo_id") == axis and event.get("field") == field,
                "wrong-axis/field reply",
            )
            if field == "position":
                value = _integer(event.get("value"), 500, 2500, "reply position")
                expected = "#%03dP%04d!" % (axis, value)
            elif field == "mode":
                value = _integer(event.get("value"), 0, 9, "reply mode")
                expected = "#%03dMOD%d!" % (axis, value)
            else:
                expected = "#%03d@ %s!" % (axis, _text(event.get("value"), "firmware"))
            _need(
                bytes(raw).strip(b"\r\n") == expected.encode("ascii"),
                "parsed reply differs from raw RX",
            )
            _time(event.get("received_at"))
            replies.append((line, event))
            query = None
            raw.clear()
    _need(query is None and pending_write is None, "unfinished serial transaction")
    _need(sum(e.get("event") == "closed" for e in events) == 1, "session not closed exactly once")
    _need(
        not any(e.get("event") in ("fault", "logging_error", "error") for e in events),
        "fault in session",
    )
    return replies, motions


def _sample_key(axis, sample, position_key="value"):
    _integer(axis, 0, 5, "servo_id")
    _integer(sample.get(position_key), 500, 2500, "position")
    _time(sample.get("received_at"))
    return axis, sample[position_key], sample["received_at"]


def inspect_session(root, directory):
    """Read one completed unified session; return consistent candidate data only."""
    root = Path(root).resolve()
    directory = _path(root, directory)
    files = [
        directory / name
        for name in ("cli_result.json", "session.json", "captured_poses.json", "events.jsonl")
    ]
    hashes = [file_evidence(root, path) for path in files]
    cli, session, stored = (_load(path) for path in files[:3])
    events = _events(files[3])
    replies, motions = _raw_replies(events)
    _need(cli.get("source") == session.get("source") == "HARDWARE", "source mismatch")
    _need(
        cli.get("closed") is True
        and session.get("closed") is True
        and cli.get("error") is None
        and session.get("faulted") is False
        and session.get("logging_errors") == [],
        "incomplete/failed session",
    )
    _need(
        all(session.get(k) == v for k, v in cli.get("driver", {}).items()) and cli.get("driver"),
        "CLI driver snapshot differs from session",
    )
    pose_events = [e for e in events if e.get("event") == "pose_capture"]
    poses = [e["pose"] for e in pose_events]
    _need(
        type(stored.get("schema_version")) is int and stored["schema_version"] == 1,
        "unsupported pose schema",
    )
    _need(stored.get("poses") == cli.get("poses") == poses, "saved/CLI/event poses disagree")
    _need(len({p.get("name") for p in poses}) == len(poses), "duplicate pose names")
    position_replies = Counter(
        _sample_key(e["servo_id"], e) for _, e in replies if e["field"] == "position"
    )
    observed = Counter()

    def check_state(state):
        _need(
            state.get("source") == "HARDWARE"
            and state.get("status") == "OK"
            and state.get("simultaneous") is False,
            "invalid/seemingly simultaneous state",
        )
        servos = state.get("servos", [])
        _need(
            len(servos) == 6 and [s.get("servo_id") for s in servos] == list(range(6)),
            "state needs ordered six axes",
        )
        keys = []
        for servo in servos:
            _need(servo.get("status") == "OK", "failed servo state")
            key = _sample_key(servo["servo_id"], servo, "position_p")
            _need(key in position_replies, "state position lacks raw-backed reply")
            keys.append(key)
        return keys

    for event in events:
        if event.get("event") == "arm_state":
            observed.update(check_state(event["state"]))
    pose_rows = []
    for event in pose_events:
        pose, states = event["pose"], event.get("states", [])
        _need(
            pose.get("source") == "HARDWARE" and pose.get("verified") is False,
            "inventory accepts unverified captures only",
        )
        _text(pose.get("name"), "pose name")
        _need(len(states) == 3, "pose needs three state rounds")
        times = [_time(state.get("finished_at")) for state in states]
        _need(
            all(a < b for a, b in zip(times, times[1:])),
            "pose rounds must have distinct increasing times",
        )
        for state in states:
            check_state(state)
            _need(
                any(e.get("event") == "arm_state" and e.get("state") == state for e in events),
                "pose state missing from state events",
            )
        spans = [
            max(s["servos"][i]["position_p"] for s in states)
            - min(s["servos"][i]["position_p"] for s in states)
            for i in range(6)
        ]
        _need(max(spans) <= 2, "pose not stable within 2P")
        _need(
            pose.get("positions_p") == [s["position_p"] for s in states[-1]["servos"]]
            and pose.get("captured_at") == states[-1].get("finished_at"),
            "pose does not equal final capture",
        )
        pose_rows.append({**pose, "spans_p": spans, "data_consistent": True})
    steps = cli.get("steps", [])
    _need(
        [s["result"] for s in steps if s["step"]["kind"] == "capture_pose"] == poses,
        "CLI capture steps disagree",
    )
    for step in steps:
        if step["step"]["kind"] == "read_state":
            check_state(step["result"])
            _need(
                any(
                    e.get("event") == "arm_state" and e.get("state") == step["result"]
                    for e in events
                ),
                "CLI state missing from event log",
            )
    actions = [s["result"] for s in steps if s["step"]["kind"] in ("move_joint", "grip")]
    results = [e["result"] for e in events if e.get("event") == "move_result"]
    plans = session.get("planned_moves", [])
    _need(
        actions == session.get("actions")
        and len(actions) == len(results) == len(motions) == len(plans),
        "action/plan/event counts disagree",
    )
    for key in ("planned_count", "consumed_count", "motion_attempts"):
        _need(
            type(session.get(key)) is int and session[key] == len(actions),
            "invalid session counters",
        )
    _need(session.get("remaining_count") == 0, "unconsumed plan")
    _need(len(list(directory.glob("attempt_*.json"))) == len(actions), "budget count mismatch")
    residuals = []
    for index, (action, logged, motion, plan) in enumerate(
        zip(actions, results, motions, plans), 1
    ):
        _need(
            all(
                key in logged
                for key in (
                    "source",
                    "servo_id",
                    "target_p",
                    "duration_ms",
                    "start_p",
                    "final_p",
                    "error_p",
                    "start_samples",
                    "feedback",
                    "final_verification",
                    "response_pass",
                    "position_reached",
                    "motion_attempted",
                    "motion_write_complete",
                )
            ),
            "incomplete action result event",
        )
        _need(all(action.get(k) == v for k, v in logged.items()), "action changed from event log")
        _need(
            action.get("source") == "HARDWARE"
            and action.get("motion_attempted") is True
            and action.get("motion_write_complete") is True
            and action.get("error") is None,
            "action is not a completed physical write",
        )
        spec = {k: action.get(k) for k in ("servo_id", "target_p", "duration_ms")}
        _need(spec == plan, "action differs from plan")
        axis = _integer(spec["servo_id"], 0, 5, "servo_id")
        _integer(spec["target_p"], 500, 2500, "target_p")
        _integer(spec["duration_ms"], 20, 9999, "duration_ms")
        command = "#%03dP%04dT%04d!" % (axis, spec["target_p"], spec["duration_ms"])
        _need(motion.get("command") == command, "motion command differs from plan")
        budget_path = directory / ("attempt_%02d.json" % index)
        budget = _load(budget_path)
        hashes.append(file_evidence(root, budget_path))
        _need(
            budget.get("kind") == "MOTION"
            and budget.get("spec") == spec
            and budget.get("command") == command
            and budget.get("plan_index") == index
            and budget.get("execution_id") == action.get("execution_id")
            and budget.get("step_id") == action.get("step_id")
            and _time(budget.get("time")) <= _time(motion.get("time")),
            "budget absent/late/inconsistent",
        )
        starts, feedback, final = (
            action.get("start_samples", []),
            action.get("feedback", []),
            action.get("final_verification"),
        )
        _need(len(starts) == 3 and feedback and type(final) is dict, "missing motion samples")
        for sample in [*starts, *feedback, final]:
            key = _sample_key(axis, sample)
            _need(key in position_replies, "motion sample lacks raw-backed reply")
            observed[key] += 1
        _need(
            max(s["value"] for s in starts) - min(s["value"] for s in starts) <= 2,
            "unstable motion start",
        )
        _need(
            action.get("start_p") == starts[-1]["value"]
            and action.get("final_p") == final["value"],
            "motion endpoints differ from samples",
        )
        _need(action.get("error_p") == spec["target_p"] - final["value"], "wrong residual")
        _need(
            action.get("observed_delta_p") == final["value"] - action["start_p"],
            "wrong observed delta",
        )
        _need(
            type(action.get("response_pass")) is bool
            and type(action.get("position_reached")) is bool,
            "motion outcome flags must be booleans",
        )
        _need(
            not action["position_reached"] or abs(action["error_p"]) <= 3,
            "precision claim conflicts with final residual",
        )
        residuals.append(
            {
                **spec,
                "source": "HARDWARE",
                "start_p": action["start_p"],
                "final_p": final["value"],
                "error_p": action["error_p"],
                "response_pass": action.get("response_pass"),
                "position_reached": action.get("position_reached"),
            }
        )
    _need(
        observed == position_replies, "raw-backed positions and result/state observations disagree"
    )
    return {
        "directory": directory.relative_to(root).as_posix(),
        "files": hashes,
        "poses": pose_rows,
        "motions": residuals,
        "query_count": len(replies),
        "position_count": sum(position_replies.values()),
        "closed": True,
        "data_consistent": True,
        "verified": False,
    }


def inventory(root, directories):
    """No recursive discovery: callers explicitly select sessions, once each."""
    paths = [_path(root, p) for p in directories]
    _need(paths and len(set(paths)) == len(paths), "sessions must be nonempty and unique")
    sessions = [inspect_session(root, path) for path in paths]
    rows = [motion for s in sessions for motion in s["motions"]]
    return {
        "schema_version": 1,
        "status": "DATA_CONSISTENT",
        "candidate_only": True,
        "motion_authorized": False,
        "physical_accuracy_verified": False,
        "pose_count": sum(len(s["poses"]) for s in sessions),
        "motion_count": len(rows),
        "sessions": sessions,
        "residuals": summarize_directional_residuals(rows) if rows else None,
        "missing": [
            "external angle/width measurements",
            "geometry measurements",
            "verified empty-load segmented paths",
        ],
        "limitation": "File consistency and hashes do not independently prove physical authenticity.",
    }


def templates():
    base = {
        "schema_version": 1,
        "dataset_id": None,
        "p_kind": "observed_feedback",
        "measurement_method": None,
        "reference": None,
        "samples": [],
    }
    return {
        "joint_angle.json": {**base, "kind": "joint_angle"},
        "gripper_width.json": {**base, "kind": "gripper_width"},
        "geometry.json": {
            "schema_version": 1,
            "kind": "geometry",
            "verified": False,
            "base_frame_definition": None,
            "joint_axes": [],
            "link_lengths_mm": [],
            "tool_offset_mm": None,
            "measurement_evidence": [],
        },
    }


def validate_measurements(root, filename):
    """Validate submitted evidence; output a candidate fit, never a usable motion calibration."""
    document = _load(_path(root, filename))
    _need(
        type(document.get("schema_version")) is int and document["schema_version"] == 1,
        "unsupported measurement schema",
    )
    _need(document.get("kind") in ("joint_angle", "gripper_width"), "unknown measurement kind")
    _need(
        document.get("p_kind") == "observed_feedback", "P must be observed feedback, not a target"
    )
    _text(document.get("dataset_id"), "dataset_id")
    _text(document.get("measurement_method"), "measurement_method")
    _text(document.get("reference"), "measurement reference")
    rows = document.get("samples")
    _need(type(rows) is list and len(rows) >= 3, "at least three external measurements are needed")
    cached = {}
    refs = set()
    evidence_files = [file_evidence(root, filename)]
    value_key = "angle_deg" if document["kind"] == "joint_angle" else "width_mm"
    for row in rows:
        _need(
            type(row) is dict
            and row.get("source") == "HARDWARE"
            and row.get("external_measurement") is True,
            "external HARDWARE measurement declaration required",
        )
        axis = _integer(row.get("servo_id"), 0, 4 if value_key == "angle_deg" else 5, "servo_id")
        _need(value_key != "width_mm" or axis == 5, "width measurements belong to gripper 005")
        _integer(row.get("p"), 500, 2500, "measured p")
        _need(
            type(row.get(value_key)) in (int, float) and math.isfinite(row[value_key]),
            "measurement must be finite",
        )
        _time(row.get("observed_at"))
        feedback_ref = row.get("feedback_evidence")
        feedback_path = _checked_evidence(root, feedback_ref)
        _need(feedback_path.name == "events.jsonl", "feedback must reference unified events.jsonl")
        if feedback_path not in cached:
            inspect_session(root, feedback_path.parent)
            event_rows = _events(feedback_path)
            cached[feedback_path] = dict(_raw_replies(event_rows)[0])
        line = _integer(feedback_ref.get("line"), 1, 10**9, "feedback line")
        identity = (feedback_path, line)
        _need(identity not in refs, "feedback evidence reused")
        refs.add(identity)
        reply = cached[feedback_path].get(line)
        _need(
            reply is not None
            and reply.get("field") == "position"
            and reply.get("servo_id") == axis
            and reply.get("value") == row["p"]
            and reply.get("received_at") == row["observed_at"],
            "measurement P/time is not the referenced observed feedback",
        )
        measurement_path = _checked_evidence(root, row.get("measurement_evidence"))
        _need(
            measurement_path != feedback_path and measurement_path.suffix.lower() == ".json",
            "external measurement requires a separate JSON observation record",
        )
        measured = _load(measurement_path)
        for key in ("sample_id", "servo_id", "p", value_key, "observed_at"):
            _need(measured.get(key) == row.get(key), "external observation record mismatch: " + key)
        _need(
            measured.get("external_measurement") is True
            and measured.get("p_kind") == "observed_feedback",
            "external observation record lacks measured feedback declaration",
        )
        _text(measured.get("method"), "external observation method")
        evidence_files.extend(
            [file_evidence(root, feedback_path), file_evidence(root, measurement_path)]
        )
    model = fit_angle_mapping(rows) if value_key == "angle_deg" else build_gripper_table(rows)
    return {
        "schema_version": 1,
        "status": "CANDIDATE_FIT",
        "candidate_only": True,
        "motion_authorized": False,
        "physical_accuracy_verified": False,
        "verified": False,
        "model": model,
        "evidence": list({f["path"]: f for f in evidence_files}.values()),
        "limitation": "External observation declarations require human review; fit residuals are not held-out accuracy.",
    }
