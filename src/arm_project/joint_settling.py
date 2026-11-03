"""Fixed-time shoulder repeatability experiment, independent of serial setup."""

from statistics import median

from arm_project.unified_arm import MoveSpec

CHECKPOINTS_S = (4.0, 6.0, 8.0, 10.0)
OFFSETS_S = tuple(t + d for t in CHECKPOINTS_S for d in (0.0, 0.1, 0.2))
MOVES = tuple(MoveSpec(1, target, 2000) for _ in range(3) for target in (1530, 1500))


def summarize_trials(trials):
    """Compare original final values and medians at predeclared host time windows.

    An incomplete/late experiment never passes. No tolerance, command or driver
    result is rewritten; an eventual small span is not external positioning proof.
    """
    groups = {str(p): {str(t): [] for t in CHECKPOINTS_S} for p in (1530, 1500)}
    valid = isinstance(trials, list) and len(trials) == len(MOVES)
    rows = []
    for index, trial in enumerate(trials):
        move = trial["move"]
        anchor = move.get("motion_write_monotonic_s")
        samples = trial.get("observation", {}).get("samples", [])
        trial_ok = (
            index < len(MOVES)
            and move.get("servo_id") == 1
            and move.get("target_p") == MOVES[index].target_p
            and move.get("duration_ms") == 2000
            and move.get("motion_write_complete") is True
            and move.get("response_pass") is True
            and not move.get("error")
            and type(anchor) in (int, float)
            and len(samples) == len(OFFSETS_S)
        )
        windows = []
        if trial_ok:
            for n, checkpoint in enumerate(CHECKPOINTS_S):
                selected = samples[n * 3 : n * 3 + 3]
                timing_ok = all(
                    abs(s["deadline_monotonic_s"] - anchor - expected) < 1e-6
                    and -1e-6 <= s["received_monotonic_s"] - s["deadline_monotonic_s"] <= 0.2
                    for s, expected in zip(selected, OFFSETS_S[n * 3 : n * 3 + 3])
                )
                values = [s["value"] for s in selected]
                span = max(values) - min(values)
                value = median(values)
                trial_ok = trial_ok and timing_ok and span <= 2
                windows.append(
                    {
                        "offset_s": checkpoint,
                        "values_p": values,
                        "median_p": value,
                        "span_p": span,
                        "timing_ok": timing_ok,
                        "target_minus_median_p": move["target_p"] - value,
                    }
                )
                groups[str(move["target_p"])][str(checkpoint)].append(value)
        valid = valid and trial_ok
        rows.append(
            {
                "trial": index + 1,
                "target_p": move.get("target_p"),
                "original_final_p": move.get("final_p"),
                "sampling_valid": trial_ok,
                "windows": windows,
            }
        )
    comparisons = []
    for target, windows in groups.items():
        for offset, values in windows.items():
            span = max(values) - min(values) if values else None
            comparisons.append(
                {
                    "target_p": int(target),
                    "offset_s": float(offset),
                    "medians_p": values,
                    "span_p": span,
                    "repeatability_pass": bool(valid and len(values) == 3 and span <= 3),
                }
            )
    return {
        "schema_version": 1,
        "sampling_valid": valid,
        "trials": rows,
        "comparisons": comparisons,
        "repeatability_limit_p": 3,
        "physical_positioning_verified": False,
        "compensation_authorized": False,
    }


def run_experiment(driver, run_id, *, progress=lambda value: None):
    """Execute exactly MOVES, sampling after each; fail before the next on any fault."""
    initial = driver.read_state()
    if initial.get("status") != "OK":
        raise RuntimeError("initial six-axis state incomplete")
    trials = []
    reference = [s["position_p"] for s in initial["servos"]]
    for index, spec in enumerate(MOVES):
        move = driver.move_joint(
            spec.servo_id,
            spec.target_p,
            spec.duration_ms,
            execution_id=run_id,
            step_id="settling_%02d" % (index + 1),
        )
        trial = {"move": move}
        trials.append(trial)
        if (
            not move.get("response_pass")
            or not move.get("motion_write_complete")
            or move.get("error")
        ):
            raise RuntimeError("motion response failed; remaining moves cancelled")
        anchor = move["motion_write_monotonic_s"]
        trial["observation"] = driver.sample_joint_at(1, [anchor + t for t in OFFSETS_S])
        state = driver.read_state()
        trial["state_after"] = state
        if state.get("status") != "OK":
            raise RuntimeError("post-observation six-axis state incomplete")
        current = [s["position_p"] for s in state["servos"]]
        if any(abs(current[i] - reference[i]) > 3 for i in (0, 2, 3, 4, 5)):
            raise RuntimeError("uncommanded-axis feedback changed; remaining moves cancelled")
        progress(trial)
    return {"initial_state": initial, "trials": trials, "analysis": summarize_trials(trials)}
