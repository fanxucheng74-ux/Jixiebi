"""Offline tests: synthetic records and clearly labelled test measurements.

No serial/camera imports or access; mutations are confined to temporary copies.
"""

from contextlib import redirect_stdout
from copy import deepcopy
import io
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from uuid import uuid4

from arm_project.calibration_evidence import (
    EvidenceError,
    file_evidence,
    inventory,
    templates,
    validate_measurements,
)
from arm_project.cli.calibration_prepare import main
from arm_project.paths import PROJECT_ROOT


SESSIONS = (
    "runs/unified/unified_read_capture_20261031_01",
    "runs/unified/unified_interface_20261031_01",
)


def load(path):
    return json.loads(path.read_text(encoding="utf-8"))


def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")


class CalibrationEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.parent = Path(tempfile.gettempdir()).resolve()
        self.root = self.parent / ("arm_calibration_test_" + uuid4().hex)
        self.root.mkdir()
        for session in SESSIONS:
            fixture = PROJECT_ROOT / "tests/fixtures/unified" / Path(session).name
            shutil.copytree(fixture, self.root / session)
        self.session = self.root / SESSIONS[1]

    def tearDown(self):
        target = self.root.resolve()
        if target.parent != self.parent or not target.name.startswith("arm_calibration_test_"):
            raise RuntimeError("temporary cleanup escaped test root")
        shutil.rmtree(target)

    def mutate_events(self, mutate):
        path = self.session / "events.jsonl"
        events = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
        mutate(events)
        path.write_text("".join(json.dumps(row) + "\n" for row in events), encoding="utf-8")

    def measurement_fixture(self, width=False):
        """Artificial external measurements for validation tests, NEVER production data."""
        axis = 5 if width else 4
        key = "width_mm" if width else "angle_deg"
        path = self.session / "events.jsonl"
        distinct = {}
        for line, text in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            event = json.loads(text)
            if (
                event.get("event") == "reply"
                and event.get("field") == "position"
                and event.get("servo_id") == axis
            ):
                distinct.setdefault(event["value"], (line, event))
        picks = sorted(distinct.items())[:3]
        samples = []
        for index, (position, (line, event)) in enumerate(picks):
            sample = {
                "source": "HARDWARE",
                "external_measurement": True,
                "sample_id": "TEST_ONLY_%d" % index,
                "servo_id": axis,
                "p": position,
                key: float(10 + index * 5),
                "observed_at": event["received_at"],
                "feedback_evidence": {**file_evidence(self.root, path), "line": line},
            }
            observation = {k: sample[k] for k in ("sample_id", "servo_id", "p", key, "observed_at")}
            observation.update(
                external_measurement=True,
                p_kind="observed_feedback",
                method="SYNTHETIC TEST FIXTURE",
            )
            observation_path = self.root / ("TEST_ONLY_measurement_%d.json" % index)
            write(observation_path, observation)
            sample["measurement_evidence"] = file_evidence(self.root, observation_path)
            samples.append(sample)
        document = {
            "schema_version": 1,
            "dataset_id": "TEST_ONLY",
            "kind": "gripper_width" if width else "joint_angle",
            "p_kind": "observed_feedback",
            "reference": "SYNTHETIC TEST REFERENCE",
            "measurement_method": "SYNTHETIC TEST FIXTURE",
            "samples": samples,
        }
        dataset = self.root / "TEST_ONLY_dataset.json"
        write(dataset, document)
        return dataset, document

    def test_synthetic_session_records_have_four_poses_four_motions_without_modification(self):
        before = {
            str(p.relative_to(self.root)): p.read_bytes()
            for p in self.root.rglob("*")
            if p.is_file()
        }
        report = inventory(self.root, SESSIONS)
        self.assertEqual((report["pose_count"], report["motion_count"]), (4, 4))
        self.assertEqual([s["query_count"] for s in report["sessions"]], [24, 134])
        self.assertEqual([s["position_count"] for s in report["sessions"]], [24, 126])
        self.assertEqual([m["error_p"] for m in report["sessions"][1]["motions"]], [7, -5, -17, 20])
        self.assertTrue(all(not p["verified"] for s in report["sessions"] for p in s["poses"]))
        self.assertTrue(report["candidate_only"])
        self.assertFalse(report["motion_authorized"])
        self.assertFalse(report["physical_accuracy_verified"])
        after = {
            str(p.relative_to(self.root)): p.read_bytes()
            for p in self.root.rglob("*")
            if p.is_file()
        }
        self.assertEqual(before, after)

    def test_duplicate_session_and_path_escape_rejected(self):
        for paths in ([SESSIONS[0], SESSIONS[0]], ["../outside"], []):
            with self.subTest(paths=paths), self.assertRaises(EvidenceError):
                inventory(self.root, paths)

    def test_raw_byte_mutation_rejected(self):
        def mutate(events):
            event = next(
                e for e in events if e.get("event") == "rx" and len(e.get("rx_hex", "")) > 2
            )
            event["rx_hex"] = event["rx_hex"].replace("35", "34", 1)

        self.mutate_events(mutate)
        with self.assertRaisesRegex(EvidenceError, "raw RX"):
            inventory(self.root, SESSIONS)

    def test_saved_pose_mutation_rejected(self):
        path = self.session / "captured_poses.json"
        saved = load(path)
        saved["poses"][0]["positions_p"][0] += 1
        write(path, saved)
        with self.assertRaisesRegex(EvidenceError, "poses disagree"):
            inventory(self.root, SESSIONS)

    def test_missing_axis_never_passes(self):
        def mutate(events):
            event = next(e for e in events if e.get("event") == "pose_capture")
            event["states"][0]["servos"].pop()

        self.mutate_events(mutate)
        with self.assertRaises(EvidenceError):
            inventory(self.root, SESSIONS)

    def test_session_action_and_late_budget_rejected(self):
        path = self.session / "session.json"
        original = load(path)
        altered = deepcopy(original)
        altered["actions"][0]["final_p"] += 1
        write(path, altered)
        with self.assertRaisesRegex(EvidenceError, "counts disagree"):
            inventory(self.root, SESSIONS)
        write(path, original)
        path = self.session / "attempt_01.json"
        budget = load(path)
        budget["time"] = "2099-01-01T00:00:00+08:00"
        write(path, budget)
        with self.assertRaisesRegex(EvidenceError, "budget"):
            inventory(self.root, SESSIONS)

    def test_simulation_source_rejected(self):
        def mutate(events):
            next(e for e in events if e["event"] == "opened")["source"] = "SIMULATION"

        self.mutate_events(mutate)
        with self.assertRaisesRegex(EvidenceError, "hardware"):
            inventory(self.root, SESSIONS)

    def test_short_write_rejected(self):
        def mutate(events):
            event = next(
                e
                for e in events
                if e.get("event") == "wire_write_returned" and e.get("kind") == "motion"
            )
            event["bytes_written"] -= 1

        self.mutate_events(mutate)
        with self.assertRaisesRegex(EvidenceError, "short/incomplete"):
            inventory(self.root, SESSIONS)

    def test_reusing_one_state_three_times_is_not_a_stable_capture(self):
        def mutate(events):
            event = next(e for e in events if e.get("event") == "pose_capture")
            event["states"] = [deepcopy(event["states"][-1]) for _ in range(3)]

        self.mutate_events(mutate)
        with self.assertRaisesRegex(EvidenceError, "distinct increasing"):
            inventory(self.root, SESSIONS)

    def test_nonobject_measurement_json_returns_structured_cli_error(self):
        write(self.root / "invalid.json", [])
        stream = io.StringIO()
        with redirect_stdout(stream):
            self.assertEqual(main(["--root", str(self.root), "validate", "invalid.json"]), 1)
        self.assertEqual(json.loads(stream.getvalue())["status"], "INVALID_OR_INCOMPLETE")

    def test_complete_external_evidence_only_produces_candidate_fit(self):
        dataset, _ = self.measurement_fixture()
        report = validate_measurements(self.root, dataset)
        self.assertEqual(report["status"], "CANDIDATE_FIT")
        self.assertFalse(report["verified"])
        self.assertFalse(report["motion_authorized"])
        self.assertFalse(report["physical_accuracy_verified"])
        self.assertEqual(report["model"]["servo_id"], 4)
        self.assertEqual(report["model"]["sample_count"], 3)

    def test_width_candidate_does_not_claim_force_or_grasp(self):
        dataset, _ = self.measurement_fixture(width=True)
        report = validate_measurements(self.root, dataset)
        self.assertEqual(report["model"]["servo_id"], 5)
        self.assertFalse(report["model"]["force_or_grasp_measurement"])
        self.assertFalse(report["model"]["defines_safe_motion_limits"])

    def test_target_p_and_wrong_reply_or_timestamp_cannot_substitute_feedback(self):
        dataset, document = self.measurement_fixture()
        changes = [
            {"p_kind": "target_p"},
            {"sample_p": 1540},
            {"observed_at": "2026-10-31T00:00:00+08:00"},
        ]
        for change in changes:
            altered = deepcopy(document)
            if "p_kind" in change:
                altered.update(change)
            elif "sample_p" in change:
                altered["samples"][0]["p"] = change["sample_p"]
            else:
                altered["samples"][0].update(change)
            write(dataset, altered)
            with self.subTest(change=change), self.assertRaises(EvidenceError):
                validate_measurements(self.root, dataset)

    def test_absent_or_altered_external_evidence_rejected(self):
        dataset, original = self.measurement_fixture()
        for change in (
            {"measurement_evidence": None},
            {"feedback_evidence": {"path": "../escape", "sha256": "x"}},
            {
                "measurement_evidence": {
                    **original["samples"][0]["measurement_evidence"],
                    "sha256": "0" * 64,
                }
            },
        ):
            document = deepcopy(original)
            document["samples"][0].update(change)
            write(dataset, document)
            with self.subTest(change=change), self.assertRaises(EvidenceError):
                validate_measurements(self.root, dataset)

    def test_duplicate_boolean_nan_or_mixed_axis_samples_rejected(self):
        dataset, original = self.measurement_fixture()
        mutations = [
            lambda d: d["samples"].__setitem__(1, deepcopy(d["samples"][0])),
            lambda d: d["samples"][0].update(p=True),
            lambda d: d["samples"][0].update(angle_deg=float("nan")),
            lambda d: d["samples"][0].update(servo_id=3),
        ]
        for mutate in mutations:
            document = deepcopy(original)
            mutate(document)
            write(dataset, document)
            with self.assertRaises(EvidenceError):
                validate_measurements(self.root, dataset)

    def test_template_cli_creates_empty_templates_and_refuses_overwrite(self):
        directory = self.root / "empty_templates"
        with redirect_stdout(io.StringIO()):
            self.assertEqual(main(["--root", str(self.root), "template", str(directory)]), 0)
            self.assertEqual(main(["--root", str(self.root), "template", str(directory)]), 1)
        for name, expected in templates().items():
            self.assertEqual(load(directory / name), expected)
        with self.assertRaises(EvidenceError):
            validate_measurements(self.root, directory / "joint_angle.json")

    def test_inventory_cli_writes_new_report_and_refuses_overwrite(self):
        output = self.root / "inventory.json"
        argv = ["--root", str(self.root), "inventory", *SESSIONS, "--output", str(output)]
        with redirect_stdout(io.StringIO()):
            self.assertEqual(main(argv), 0)
            self.assertEqual(main(argv), 1)
        self.assertEqual(load(output)["motion_count"], 4)
        with redirect_stdout(io.StringIO()):
            self.assertEqual(main(["--root", str(self.root), "validate", "absent.json"]), 1)


if __name__ == "__main__":
    unittest.main()
