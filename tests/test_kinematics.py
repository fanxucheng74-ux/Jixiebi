"""Mathematical nominal-model fixtures; no actual dimensions or hardware tests."""

from contextlib import redirect_stdout
from dataclasses import FrozenInstanceError
import io
import hashlib
import json
import math
from pathlib import Path
import random
import shutil
import tempfile
import unittest
from unittest.mock import patch
from uuid import uuid4

from arm_project.kinematics import (
    Geometry,
    Joints,
    Pose,
    forward,
    inverse,
    from_nominal_pulses,
    to_nominal_pulses,
    wrap_degrees,
)
from arm_project.cli.inspect_kinematics import inspect_model, main
from arm_project.paths import PROJECT_ROOT


GEOMETRY = Geometry(170, 105, 75, 185)  # Manufacturer nominal fixture only.
LIMITS = {
    "yaw_deg": [-90, 90],
    "shoulder_deg": [0, 180],
    "elbow_flex_deg": [-135, 135],
    "wrist_deg": [-90, 90],
}


def config():
    return json.loads(
        (PROJECT_ROOT / "configs/kinematics.vendor_nominal.json").read_text(encoding="utf-8")
    )


class KinematicTests(unittest.TestCase):
    def assert_pose(self, actual, expected, tolerance=1e-5):
        self.assertLessEqual(
            math.dist(
                (actual.x_mm, actual.y_mm, actual.z_mm),
                (expected.x_mm, expected.y_mm, expected.z_mm),
            ),
            tolerance,
        )
        self.assertAlmostEqual(wrap_degrees(actual.pitch_deg - expected.pitch_deg), 0, places=6)

    def test_vertical_and_horizontal_have_known_lengths(self):
        self.assert_pose(forward(GEOMETRY, Joints(0, 90, 0, 0)), Pose(0, 0, 535, 90))
        self.assert_pose(forward(GEOMETRY, Joints(0, 0, 0, 0)), Pose(0, 365, 170, 0))

    def test_positive_yaw_points_towards_positive_x(self):
        self.assert_pose(forward(GEOMETRY, Joints(90, 0, 0, 0)), Pose(365, 0, 170, 0))
        self.assert_pose(forward(GEOMETRY, Joints(-90, 0, 0, 0)), Pose(-365, 0, 170, 0))

    def test_positive_elbow_flexion_reduces_forearm_elevation(self):
        self.assert_pose(forward(GEOMETRY, Joints(0, 90, 90, 0)), Pose(0, 260, 275, 0))

    def test_forward_inverse_roundtrip_over_varied_configurations(self):
        randomizer = random.Random(20261101)
        for _ in range(60):
            q = Joints(
                randomizer.uniform(-80, 80),
                randomizer.uniform(15, 160),
                randomizer.uniform(-100, 100),
                randomizer.uniform(-60, 60),
            )
            pose = forward(GEOMETRY, q)
            solutions = inverse(GEOMETRY, pose, LIMITS, base_hint_deg=q.yaw_deg)
            self.assertTrue(solutions, (q, pose))
            for candidate in solutions:
                self.assert_pose(forward(GEOMETRY, candidate), pose)
            self.assertTrue(
                any(
                    max(
                        abs(wrap_degrees(a - b)) for a, b in zip(vars(q).values(), vars(s).values())
                    )
                    < 1e-5
                    for s in solutions
                )
            )

    def test_elbow_branches_are_independent_and_reconstruct_target(self):
        pose = forward(GEOMETRY, Joints(0, 80, 50, 10))
        solutions = inverse(GEOMETRY, pose, LIMITS)
        self.assertEqual(len(solutions), 2)
        self.assertEqual({math.copysign(1, s.elbow_flex_deg) for s in solutions}, {-1, 1})

    def test_behind_base_target_uses_signed_radial_branch(self):
        original = Joints(-45, 110, 15, -5)
        pose = forward(GEOMETRY, original)
        self.assertLess(pose.y_mm, 0)
        solutions = inverse(GEOMETRY, pose, LIMITS)
        self.assertTrue(solutions)
        for solution in solutions:
            self.assert_pose(forward(GEOMETRY, solution), pose)

    def test_base_axis_uses_explicit_hint_without_division_by_zero(self):
        solutions = inverse(GEOMETRY, Pose(0, 0, 535, 90), LIMITS, base_hint_deg=30)
        self.assertEqual(len(solutions), 1)
        self.assertAlmostEqual(solutions[0].yaw_deg, 30)

    def test_unreachable_and_zero_wrist_center_return_no_stale_solution(self):
        self.assertTrue(inverse(GEOMETRY, Pose(0, 0, 535, 90), LIMITS))
        for pose in (Pose(10000, 10000, 10000, 0), Pose(0, 0, 0, 0), Pose(0, 0, 355, 90)):
            self.assertEqual(inverse(GEOMETRY, pose, LIMITS), ())
        self.assertTrue(inverse(GEOMETRY, Pose(0, 0, 535, 90), LIMITS))

    def test_models_and_results_have_no_mutable_shared_state(self):
        other = Geometry(110, 105, 75, 185)
        self.assertAlmostEqual(
            forward(GEOMETRY, Joints(0, 90, 0, 0)).z_mm - forward(other, Joints(0, 90, 0, 0)).z_mm,
            60,
        )
        solution = inverse(GEOMETRY, Pose(0, 0, 535, 90), LIMITS)[0]
        with self.assertRaises(FrozenInstanceError):
            solution.yaw_deg = 20

    def test_nominal_pulse_sign_offset_and_integer_roundtrip(self):
        q = from_nominal_pulses([1500, 1500, 1500, 1500])
        self.assertEqual(q, Joints(0, 90, 0, 0))
        q = from_nominal_pulses([1400, 1600, 1700, 1300])
        self.assertAlmostEqual(q.yaw_deg, 13.5)
        self.assertAlmostEqual(q.shoulder_deg, 103.5)
        self.assertEqual(to_nominal_pulses(q), (1400, 1600, 1700, 1300))

    def test_invalid_geometry_angles_positions_and_pulses_rejected(self):
        for lengths in (
            (170, 0, 75, 185),
            (170, -1, 75, 185),
            (True, 105, 75, 185),
            (170, float("nan"), 75, 185),
            (170, 1e-300, 75, 185),
        ):
            with self.subTest(lengths=lengths), self.assertRaises(ValueError):
                Geometry(*lengths)
        for value in (True, float("nan"), float("inf"), 10**1000):
            with self.subTest(value=str(value)[:30]), self.assertRaises(ValueError):
                Pose(value, 0, 0, 0)
        for pulses in ([True, 1500, 1500, 1500], [499, 1500, 1500, 1500], [1500] * 3):
            with self.assertRaises(ValueError):
                from_nominal_pulses(pulses)
        with self.assertRaises(ValueError):
            to_nominal_pulses(Joints(180, 90, 0, 0))

    def test_invalid_nominal_limits_and_base_hint_rejected(self):
        for limits in ({}, {**LIMITS, "yaw_deg": [90, -90]}, {**LIMITS, "yaw_deg": [False, 90]}):
            with self.assertRaises(ValueError):
                inverse(GEOMETRY, Pose(0, 0, 535, 90), limits)
        with self.assertRaises(ValueError):
            inverse(GEOMETRY, Pose(0, 0, 535, 90), LIMITS, base_hint_deg=float("nan"))


class KinematicReportTests(unittest.TestCase):
    def test_current_nominal_pose_roundtrip_stays_candidate_only(self):
        report = inspect_model(config(), pulses=[1507, 1502, 1494, 1495])
        self.assertEqual(report["status"], "SOLVED_GEOMETRICALLY")
        self.assertGreater(report["local_candidate_count"], 0)
        self.assertFalse(report["motion_authorized"])
        self.assertFalse(report["physical_geometry_verified"])
        self.assertFalse(report["grasp_center_verified"])
        self.assertEqual(report["held_axes"], [4, 5])

    def test_table_target_can_be_geometrically_reachable_but_outside_local_envelope(self):
        report = inspect_model(
            config(), pulses=[1507, 1502, 1494, 1495], target=[100, 200, 20], pitch=-45
        )
        self.assertTrue(report["solutions"])
        self.assertEqual(report["local_candidate_count"], 0)
        self.assertTrue(all(s["local_issues"] for s in report["solutions"]))
        self.assertFalse(report["motion_authorized"])

    def test_failure_report_cannot_retain_previous_pulses(self):
        self.assertTrue(inspect_model(config(), pulses=[1507, 1502, 1494, 1495])["solutions"])
        report = inspect_model(config(), target=[10000, 10000, 10000], pitch=0)
        self.assertEqual(report["status"], "UNREACHABLE")
        self.assertEqual(report["solutions"], [])

    def test_config_frame_endpoint_and_identity_not_silently_ignored(self):
        for key, value in (
            ("source", "MEASURED"),
            ("schema_version", True),
            ("frame", "unknown"),
            ("endpoint", "grasp_center"),
        ):
            model = config()
            model[key] = value
            with self.assertRaises(ValueError):
                inspect_model(model, pulses=[1500] * 4)

    def test_invalid_duration_does_not_create_valid_local_candidate(self):
        report = inspect_model(config(), pulses=[1507, 1502, 1494, 1495], duration_ms=0)
        self.assertEqual(report["local_candidate_count"], 0)


class KinematicCliTests(unittest.TestCase):
    def setUp(self):
        self.parent = Path(tempfile.gettempdir()).resolve()
        self.root = self.parent / ("arm_kinematic_test_" + uuid4().hex)
        self.root.mkdir()
        self.addCleanup(self.cleanup)

    def cleanup(self):
        path = self.root.resolve()
        if path.parent != self.parent or not path.name.startswith("arm_kinematic_test_"):
            raise AssertionError("cleanup path mismatch")
        shutil.rmtree(path)

    def run_cli(self, args):
        with (
            patch(
                "arm_project.unified_arm._RawSerialTransport.open",
                side_effect=AssertionError("no serial"),
            ),
            redirect_stdout(io.StringIO()),
        ):
            return main(args)

    def test_manual_input_writes_non_motion_report_and_refuses_overwrite(self):
        output = self.root / "report.json"
        args = ["--pulses", "1507", "1502", "1494", "1495", "--output", str(output)]
        self.assertEqual(self.run_cli(args), 0)
        data = json.loads(output.read_text(encoding="utf-8"))
        self.assertFalse(data["motion_authorized"])
        self.assertEqual(
            data["config_sha256"],
            hashlib.sha256(
                (PROJECT_ROOT / "configs/kinematics.vendor_nominal.json").read_bytes()
            ).hexdigest(),
        )
        self.assertEqual(self.run_cli(args), 1)

    def test_hardware_snapshot_is_not_promoted_to_measured_geometry(self):
        source = (
            PROJECT_ROOT
            / "tests/fixtures/unified/unified_read_capture_20261031_01/captured_poses.json"
        )
        output = self.root / "snapshot.json"
        self.assertEqual(self.run_cli(["--pose-file", str(source), "--output", str(output)]), 0)
        report = json.loads(output.read_text(encoding="utf-8"))
        self.assertIsNotNone(report["recorded_at"])
        self.assertEqual(
            report["pose_file_sha256"], hashlib.sha256(source.read_bytes()).hexdigest()
        )
        self.assertFalse(report["physical_angle_calibration_verified"])

    def test_missing_incomplete_target_and_malformed_pose_file_fail_closed(self):
        self.assertEqual(self.run_cli([]), 1)
        self.assertEqual(self.run_cli(["--target", "100", "200", "20"]), 1)
        self.assertEqual(self.run_cli(["--pose-name", "x", "--pulses", *["1500"] * 4]), 1)
        path = self.root / "bad.json"
        path.write_text('{"schema_version":1,"poses":[true]}', encoding="utf-8")
        self.assertEqual(self.run_cli(["--pose-file", str(path)]), 1)


if __name__ == "__main__":
    unittest.main()
