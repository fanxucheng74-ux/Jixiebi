"""Known-position planning tests are entirely offline, with explicit assumptions."""

import contextlib
import io
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from arm_project.kinematics import Geometry, forward, from_nominal_pulses
from arm_project.pick_planning import (
    PickPlanningError,
    continuous_segment,
    plan_paper_pick,
    shoulder_solutions_for_z,
    task_to_model,
    tip_for_grasp,
)

ROOT = Path(__file__).resolve().parents[1]


class PickTests(unittest.TestCase):
    def setUp(self):
        self.model = json.loads((ROOT / "configs/kinematics.vendor_nominal.json").read_text())
        self.request = json.loads((ROOT / "configs/paper_pick.example.json").read_text())
        self.start = [1491, 1497, 1503, 1496, 1533, 1508]
        self.geometry = Geometry(**self.model["geometry_mm"])

    def test_front_edge_and_near_object_edge_are_distinct(self):
        centre = plan_paper_pick(self.model, self.request, self.start)
        self.assertEqual(centre["grasp_center_task_mm"], [0, 150, 25])
        self.request["distance_reference"] = "BASE_EDGE_TO_OBJECT_NEAR_EDGE"
        near = plan_paper_pick(self.model, self.request, self.start)
        self.assertEqual(near["grasp_center_task_mm"], [0, 175, 25])
        self.assertFalse(near["motion_authorized"])

    def test_task_frame_rotation_and_vertical_offset(self):
        point = task_to_model([10, 100, 25], 90, 30)
        for a, b in zip(point, [100, -10, 55]):
            self.assertAlmostEqual(a, b)
        point = task_to_model([0, 100, 25], -90, 30)
        self.assertAlmostEqual(point[0], -100)

    def test_grasp_tcp_offset_is_not_the_closed_tip(self):
        tip = tip_for_grasp([0, 150, 25], 0, -90, 20)
        self.assertAlmostEqual(tip.z_mm, 5)
        self.assertAlmostEqual(tip.y_mm, 150)
        with self.assertRaises(ValueError):
            tip_for_grasp([0, 150, 25], 0, -90, -1)

    def test_user_home_height_aligns_one_plane_without_claiming_geometry(self):
        h = forward(self.geometry, from_nominal_pulses(self.start[:4])).z_mm
        self.request["measured_home_tip_height_mm"] = h - 10
        report = plan_paper_pick(self.model, self.request, self.start)
        self.assertAlmostEqual(report["table_z_in_model_mm"], 10)
        self.assertEqual(report["table_reference_source"], "SINGLE_USER_HOME_HEIGHT_REFERENCE")
        self.assertFalse(report["physical_geometry_verified"])
        self.assertFalse(report["motion_authorized"])

    def test_full_candidate_preserves_waits_grip_evidence_and_recorded_home(self):
        report = plan_paper_pick(self.model, self.request, self.start)
        self.assertEqual(
            [s["name"] for s in report["segments"]],
            ["approach", "descend", "lift", "return_upright"],
        )
        self.assertEqual(report["segments"][-1]["target_p"], self.start[:4])
        self.assertEqual(report["segments"][0]["held_servo_ids"], [0, 4, 5])
        self.assertEqual(report["required_opening_mm"], 56)
        self.assertIn("WAIT_OBJECT_PLACED_AND_HANDS_CLEAR", report["phases"])
        self.assertIn("WAIT_INDEPENDENT_GRASP_CONFIRMATION", report["phases"])
        self.assertFalse(report["gripper_width_to_p_verified"])
        self.assertFalse(report["grip_force_verified"])

    def test_candidate_forward_kinematics_matches_target_with_pulse_rounding(self):
        report = plan_paper_pick(self.model, self.request, self.start)
        for label in ("grasp", "approach"):
            q = from_nominal_pulses(report[label + "_pulses_p"])
            actual = forward(self.geometry, q)
            target = report[label + "_tip_model"]
            self.assertLess(
                math.dist(
                    [actual.x_mm, actual.y_mm, actual.z_mm],
                    [target[n] for n in ("x_mm", "y_mm", "z_mm")],
                ),
                1,
            )
        self.assertGreater(report["maximum_sampled_radius_mm"], report["distance_to_centre_mm"])

    def test_ar_variant_keeps_user_estimate_distinct_from_verified_measurement(self):
        model = json.loads((ROOT / "configs/kinematics.arduino_nominal.json").read_text())
        request = json.loads((ROOT / "configs/paper_pick.user_estimate.json").read_text())
        report = plan_paper_pick(model, request, self.start)
        self.assertEqual(model["geometry_mm"]["base_height_mm"], 100)
        self.assertEqual(report["grasp_center_task_mm"], [0, 250, 25])
        self.assertEqual(report["request"]["base_edge_source"], "USER_ESTIMATE")
        self.assertFalse(report["motion_authorized"])
        self.assertFalse(report["physical_geometry_verified"])
        self.assertNotEqual(model["geometry_mm"], self.model["geometry_mm"])

    def test_segments_are_one_goal_per_axis_without_probe_stop_waypoints(self):
        report = plan_paper_pick(self.model, self.request, self.start)
        for segment in report["segments"]:
            self.assertLessEqual(len(segment["commands"]), 4)
            self.assertEqual(
                len({c["servo_id"] for c in segment["commands"]}), len(segment["commands"])
            )
            self.assertTrue(all(c["duration_ms"] == 8000 for c in segment["commands"]))
            self.assertFalse(segment["motion_authorized"])
            self.assertGreaterEqual(segment["minimum_sampled_table_clearance_mm"], 5)
            samples = segment["samples"]
            for a, b in zip(samples, samples[1:]):
                self.assertLessEqual(
                    max(abs(x - y) for x, y in zip(a["pulses_p"], b["pulses_p"])), 1
                )

    def test_unreachable_or_invalid_input_never_produces_a_stale_plan(self):
        for changes in (
            {"outside_edge_distance_mm": 1000},
            {"object_radius_mm": True},
            {"base_front_edge_offset_mm": 0},
            {"duration_ms": 10000},
            {"distance_reference": "ambiguous"},
            {"base_edge_source": "guess"},
            {"extra": 1},
        ):
            req = {**self.request, **changes}
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                plan_paper_pick(self.model, req, self.start)
        with self.assertRaises(ValueError):
            plan_paper_pick(self.model, self.request, [1500] * 5)
        report = plan_paper_pick(self.model, self.request, self.start)
        self.assertEqual(report["status"], "NOMINAL_PICK_PROPOSAL")

    def test_sampled_plane_failure_rejects_a_segment(self):
        with self.assertRaises(PickPlanningError):
            continuous_segment(
                self.geometry,
                self.model["nominal_angular_limits_deg"],
                self.start[:4],
                self.start[:4],
                200,
                8000,
            )

    def test_cli_offline_output_does_not_open_hardware_or_reuse_directory(self):
        from arm_project.cli.plan_paper_pick import main

        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            state = folder / "state.json"
            state.write_text(
                json.dumps(
                    {
                        "source": "HARDWARE",
                        "error": None,
                        "state": {
                            "status": "OK",
                            "servos": [
                                {"servo_id": i, "status": "OK", "position_p": p}
                                for i, p in enumerate(self.start)
                            ],
                        },
                    }
                ),
                encoding="utf-8",
            )
            args = ["--state-file", str(state), "--output", str(folder / "out")]
            with (
                patch(
                    "arm_project.arm_link.SerialTransport.open",
                    side_effect=AssertionError("no hardware"),
                ),
                contextlib.redirect_stdout(io.StringIO()),
            ):
                self.assertEqual(main(args), 0)
                self.assertEqual(main(args), 1)
            self.assertTrue((folder / "out/preview.html").is_file())
            report = json.loads((folder / "out/proposal.json").read_text(encoding="utf-8"))
            self.assertFalse(report["motion_authorized"])

    def test_height_only_inverse_holds_other_axes_and_reports_changed_xy(self):
        model = json.loads((ROOT / "configs/kinematics.arduino_nominal.json").read_text())
        geometry = Geometry(**model["geometry_mm"])
        reference = [1491, 1435, 1812, 1033]
        answers = shoulder_solutions_for_z(
            geometry, model["nominal_angular_limits_deg"], reference, 70
        )
        self.assertTrue(answers)
        for answer in answers:
            self.assertLess(abs(answer["height_error_mm"]), 1)
            self.assertEqual(
                [answer["pulses_p"][i] for i in (0, 2, 3)], [reference[i] for i in (0, 2, 3)]
            )
            self.assertFalse(answer["motion_authorized"])
        self.assertLess(answers[0]["pulses_p"][1], reference[1])

    def test_height_inverse_unreachable_and_nonfinite_have_no_stale_result(self):
        answers = shoulder_solutions_for_z(
            self.geometry, self.model["nominal_angular_limits_deg"], self.start[:4], 200
        )
        self.assertTrue(answers)
        self.assertEqual(
            shoulder_solutions_for_z(
                self.geometry, self.model["nominal_angular_limits_deg"], self.start[:4], 10000
            ),
            (),
        )
        with self.assertRaises(ValueError):
            shoulder_solutions_for_z(
                self.geometry,
                self.model["nominal_angular_limits_deg"],
                self.start[:4],
                float("nan"),
            )


if __name__ == "__main__":
    unittest.main()
