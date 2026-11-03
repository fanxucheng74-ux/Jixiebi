"""Finite local diagnostics. These are not approved pick-and-place task paths."""

from dataclasses import asdict, dataclass
import math
from statistics import median

from arm_project.unified_arm import MoveSpec

LATE_OFFSETS_S = (10.0, 10.1, 10.2)


@dataclass(frozen=True)
class LocalStep:
    name: str
    move: MoveSpec
    approach: str = ""

    def to_dict(self):
        return {"name": self.name, "move": asdict(self.move), "approach": self.approach}


MIDPOINT_STEPS = tuple(
    LocalStep("cycle_%d_%s" % (cycle, name), MoveSpec(1, p, 2000), approach)
    for cycle in range(1, 4)
    for name, p, approach in (
        ("upper", 1530, ""),
        ("mid_from_above", 1515, "from_above"),
        ("lower", 1500, ""),
        ("mid_from_below", 1515, "from_below"),
    )
)

MIDPOINT_WIDE_STEPS = tuple(
    LocalStep("cycle_%d_%s" % (cycle, name), MoveSpec(1, p, 2000), approach)
    for cycle in range(1, 4)
    for name, p, approach in (
        ("lower", 1500, ""),
        ("mid_from_below", 1520, "from_below"),
        ("upper", 1540, ""),
        ("mid_from_above", 1520, "from_above"),
    )
)

EMPTY_PATH_STEPS = (LocalStep("prepare_lower", MoveSpec(1, 1500, 2000)),) + tuple(
    LocalStep("cycle_%d_%s" % (cycle, name), MoveSpec(axis, p, duration))
    for cycle in range(1, 3)
    for name, axis, p, duration in (
        ("shoulder_out", 1, 1530, 2000),
        ("elbow_out", 2, 1530, 2000),
        ("wrist_out", 3, 1540, 1500),
        ("roll_out", 4, 1540, 1500),
        ("roll_back", 4, 1490, 1500),
        ("wrist_back", 3, 1490, 1500),
        ("elbow_back", 2, 1490, 2000),
        ("shoulder_back", 1, 1500, 2000),
    )
)


def experiment_steps(name):
    if name == "midpoint":
        return MIDPOINT_STEPS
    if name == "midpoint_wide":
        return MIDPOINT_WIDE_STEPS
    if name == "empty_path":
        return EMPTY_PATH_STEPS
    raise ValueError("unknown local experiment")


def summarize_local(name, trials):
    expected = experiment_steps(name)
    valid = isinstance(trials, list) and len(trials) == len(expected)
    groups = {"from_above": [], "from_below": []}
    rows = []
    for index, trial in enumerate(trials):
        move = trial.get("move", {})
        samples = trial.get("observation", {}).get("samples", [])
        step_ok = index < len(expected) and trial.get("step") == expected[index].to_dict()
        anchor = move.get("motion_write_monotonic_s")
        anchor_ok = type(anchor) in (int, float) and math.isfinite(anchor)
        sample_ok = anchor_ok and len(samples) == 3
        if sample_ok:
            sample_ok = all(
                type(s.get("value")) is int
                and abs(s["deadline_monotonic_s"] - anchor - offset) < 1e-6
                and -1e-6 <= s["received_monotonic_s"] - s["deadline_monotonic_s"] <= 0.2
                for s, offset in zip(samples, LATE_OFFSETS_S)
            )
        values = [s["value"] for s in samples] if sample_ok else []
        center = median(values) if values else None
        span = max(values) - min(values) if values else None
        row_ok = (
            step_ok
            and sample_ok
            and span <= 2
            and move.get("response_pass") is True
            and move.get("motion_write_complete") is True
            and move.get("error") is None
            and trial.get("held_axes_ok") is True
        )
        if step_ok:
            spec = expected[index].move
            row_ok = row_ok and all(move.get(k) == v for k, v in asdict(spec).items())
            approach = expected[index].approach
            if approach:
                start = move.get("start_p")
                direction_ok = type(start) is int and (
                    start > spec.target_p + 3
                    if approach == "from_above"
                    else start < spec.target_p - 3
                )
                row_ok = row_ok and direction_ok
                if center is not None:
                    groups[approach].append(center)
        valid = valid and row_ok
        rows.append(
            {
                "name": trial.get("step", {}).get("name"),
                "target_p": move.get("target_p"),
                "original_final_p": move.get("final_p"),
                "late_median_p": center,
                "late_span_p": span,
                "target_minus_late_p": move["target_p"] - center if center is not None else None,
                "sampling_valid": row_ok,
                "strict_position_reached": move.get("position_reached") is True,
            }
        )
    report = {
        "schema_version": 1,
        "experiment": name,
        "data_valid": bool(valid),
        "rows": rows,
        "task_path_verified": False,
        "grasp_verified": False,
        "compensation_authorized": False,
    }
    if name in ("midpoint", "midpoint_wide"):
        report["directions"] = {}
        for direction, values in groups.items():
            span = max(values) - min(values) if values else None
            report["directions"][direction] = {
                "medians_p": values,
                "span_p": span,
                "repeatability_pass": bool(valid and len(values) == 3 and span <= 3),
            }
        report["reference_repeatability_pass"] = all(
            d["repeatability_pass"] for d in report["directions"].values()
        )
        report["directional_gap_p"] = (
            median(groups["from_above"]) - median(groups["from_below"]) if valid else None
        )
    else:
        report["local_response_sequence_complete"] = bool(valid)
    return report


def run_local(driver, name, run_id, result, *, progress=lambda value: None):
    """Store even partial trials in result; never retry a motion or promote success."""
    steps = experiment_steps(name)
    initial = driver.read_state()
    result["initial_state"] = initial
    result["trials"] = []
    if initial.get("status") != "OK":
        raise RuntimeError("incomplete initial six-axis state")
    previous = [s["position_p"] for s in initial["servos"]]
    if name == "midpoint" and previous[1] > 1510:
        raise ValueError("midpoint sequence requires a fresh lower-side shoulder start<=1510P")
    if name in ("midpoint_wide", "empty_path") and previous[1] < 1518:
        raise ValueError("sequence requires a fresh upper-side shoulder start>=1518P")
    for index, step in enumerate(steps):
        trial = {"step": step.to_dict()}
        result["trials"].append(trial)
        move = driver.move_joint(
            step.move.servo_id,
            step.move.target_p,
            step.move.duration_ms,
            execution_id=run_id,
            step_id=step.name,
        )
        trial["move"] = move
        if (
            not move.get("response_pass")
            or not move.get("motion_write_complete")
            or move.get("error")
        ):
            raise RuntimeError("step response failed; no retry or remaining moves")
        anchor = move["motion_write_monotonic_s"]
        observation = driver.sample_joint_at(
            step.move.servo_id, [anchor + t for t in LATE_OFFSETS_S]
        )
        trial["observation"] = observation
        values = [s["value"] for s in observation["samples"]]
        direction = 1 if step.move.target_p > move["start_p"] else -1
        if max(values) - min(values) > 2 or any(
            (p - move["start_p"]) * direction <= 2 for p in values
        ):
            raise RuntimeError("late feedback unstable or no longer distinctly directional")
        state = driver.read_state()
        trial["state_after"] = state
        if state.get("status") != "OK":
            raise RuntimeError("post-step six-axis state incomplete")
        current = [s["position_p"] for s in state["servos"]]
        trial["held_axes_ok"] = all(
            abs(current[i] - previous[i]) <= 3 for i in range(6) if i != step.move.servo_id
        )
        if not trial["held_axes_ok"]:
            raise RuntimeError("uncommanded-axis drift exceeds3P; remaining moves cancelled")
        trial["pose"] = {
            "name": step.name,
            "positions_p": current,
            "source": driver.source,
            "captured_at": state["finished_at"],
            "verified": False,
        }
        previous = current
        progress(trial)
    result["analysis"] = summarize_local(name, result["trials"])


def require_midpoint(path):
    """An earlier clean finite experiment gates the subsequent local path trial."""
    import json

    data = json.loads(path.read_text(encoding="utf-8"))
    driver = data.get("driver", {})
    if (
        data.get("source") != "HARDWARE"
        or driver.get("source") != "HARDWARE"
        or data.get("experiment") not in ("midpoint", "midpoint_wide")
        or data.get("error") is not None
        or driver.get("closed") is not True
        or driver.get("faulted") is not False
        or driver.get("cancel_requested") is not False
        or driver.get("logging_errors") != []
        or driver.get("planned_count") != 12
        or driver.get("remaining_count") != 0
        or driver.get("consumed_count") != 12
        or driver.get("motion_attempts") != 12
    ):
        raise ValueError("previous midpoint hardware session did not finish cleanly")
    calculated = summarize_local(data["experiment"], data.get("trials", []))
    if calculated != data.get("analysis") or not calculated["reference_repeatability_pass"]:
        raise ValueError("midpoint reference repeatability not established")
    return data
