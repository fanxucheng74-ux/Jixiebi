"""纯标定数学检查。下列HARDWARE标签是内存测试夹具，不是实物测量记录。"""

from copy import deepcopy
import math
import unittest

from arm_project.joint_calibration import (
    summarize_directional_residuals,
    fit_angle_mapping,
    angle_from_p,
    build_gripper_table,
    gripper_width_from_p,
)


def angle_samples(values=(0.0, 10.0, 20.0)):
    return [
        {
            "servo_id": 1,
            "source": "HARDWARE",
            "external_measurement": True,
            "sample_id": "fixture-%d" % i,
            "p": p,
            "angle_deg": value,
        }
        for i, (p, value) in enumerate(zip((1400, 1500, 1600), values))
    ]


def width_samples(values=(10.0, 25.0, 60.0)):
    return [
        {
            "servo_id": 5,
            "source": "HARDWARE",
            "external_measurement": True,
            "sample_id": "fixture-%d" % i,
            "p": p,
            "width_mm": value,
        }
        for i, (p, value) in enumerate(zip((1000, 1500, 2000), values))
    ]


class ResidualSummaryTests(unittest.TestCase):
    def row(self, **changes):
        return {
            "servo_id": 0,
            "source": "HARDWARE",
            "duration_ms": 1500,
            "start_p": 1520,
            "target_p": 1555,
            "final_p": 1543,
            **changes,
        }

    def test_axes_and_durations_remain_separate_and_directions_preserved(self):
        result = summarize_directional_residuals(
            [
                self.row(),
                self.row(final_p=1541),
                self.row(start_p=1543, target_p=1505, final_p=1520),
                self.row(servo_id=1, final_p=1550),
                self.row(duration_ms=3000, final_p=1540),
            ]
        )
        self.assertTrue(result["candidate_only"])
        self.assertFalse(result["physical_accuracy_verified"])
        self.assertEqual(len(result["groups"]), 3)
        base = result["groups"][0]
        self.assertEqual((base["servo_id"], base["duration_ms"]), (0, 1500))
        positive = base["directions"]["positive"]
        self.assertEqual(positive["sample_count"], 2)
        self.assertEqual(positive["error_median_p"], 13)
        self.assertEqual(positive["error_span_p"], 2)
        self.assertEqual(base["directions"]["negative"]["error_median_p"], -15)
        self.assertNotIn("compensation", result)
        self.assertNotIn("limits", result)

    def test_no_movement_and_zero_target_delta_not_discarded(self):
        result = summarize_directional_residuals(
            [self.row(final_p=1520), self.row(target_p=1520, final_p=1521)]
        )
        directions = result["groups"][0]["directions"]
        self.assertEqual(directions["positive"]["observed_delta_median_p"], 0)
        self.assertEqual(directions["zero_target_delta"]["error_median_p"], -1)

    def test_invalid_sources_fields_and_nonfinite_values_rejected(self):
        for changes in (
            {"source": "SIMULATION"},
            {"servo_id": True},
            {"servo_id": 6},
            {"duration_ms": 0},
            {"duration_ms": True},
            {"start_p": 499},
            {"target_p": 1500.0},
            {"final_p": float("nan")},
            {"final_p": True},
        ):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                summarize_directional_residuals([self.row(**changes)])
        with self.assertRaises(ValueError):
            summarize_directional_residuals([])


class AngleCalibrationTests(unittest.TestCase):
    def test_known_scale_and_offset_and_measured_interval(self):
        samples = angle_samples()
        original = deepcopy(samples)
        model = fit_angle_mapping(list(reversed(samples)))
        self.assertAlmostEqual(model["a_deg_per_p"], 0.1)
        self.assertAlmostEqual(model["b_deg"], -140)
        self.assertAlmostEqual(model["rms_error_deg"], 0)
        self.assertAlmostEqual(angle_from_p(model, 1450), 5)
        self.assertEqual((model["p_min"], model["p_max"]), (1400, 1600))
        self.assertEqual(samples, original)
        self.assertFalse(model["defines_safe_motion_limits"])

    def test_negative_scale_preserves_actual_direction(self):
        model = fit_angle_mapping(angle_samples((20, 10, 0)))
        self.assertLess(model["a_deg_per_p"], 0)
        self.assertAlmostEqual(angle_from_p(model, 1550), 5)

    def test_noisy_data_reports_residual_instead_of_claiming_accuracy(self):
        model = fit_angle_mapping(angle_samples((0, 11, 20)))
        self.assertTrue(model["candidate_only"])
        self.assertAlmostEqual(model["rms_error_deg"], math.sqrt(2 / 9))
        self.assertAlmostEqual(model["max_abs_error_deg"], 2 / 3)
        self.assertEqual(len(model["measurement_residuals_deg"]), 3)

    def test_insufficient_unattested_and_mixed_axis_data_rejected(self):
        with self.assertRaises(ValueError):
            fit_angle_mapping(angle_samples()[:2])
        for changes in (
            {"external_measurement": False},
            {"external_measurement": 1},
            {"source": "REPLAY"},
            {"servo_id": 2},
            {"sample_id": " "},
        ):
            rows = angle_samples()
            rows[1].update(changes)
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                fit_angle_mapping(rows)
        rows = angle_samples()
        del rows[0]["external_measurement"]
        with self.assertRaises(ValueError):
            fit_angle_mapping(rows)

    def test_duplicate_measurement_ids_and_positions_rejected(self):
        for key in ("sample_id", "p"):
            rows = angle_samples()
            rows[1][key] = rows[0][key]
            with self.subTest(key=key), self.assertRaises(ValueError):
                fit_angle_mapping(rows)

    def test_constant_and_zero_fit_slope_rejected(self):
        for values in ((5, 5, 5), (0, 1, 0)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                fit_angle_mapping(angle_samples(values))

    def test_nonfinite_boolean_and_extreme_angle_rejected(self):
        for invalid in (True, float("nan"), float("inf"), "1", 10**1000):
            rows = angle_samples()
            rows[1]["angle_deg"] = invalid
            with self.subTest(invalid_type=type(invalid)), self.assertRaises(ValueError):
                fit_angle_mapping(rows)

    def test_angle_conversion_refuses_extrapolation_and_bad_model(self):
        model = fit_angle_mapping(angle_samples())
        for p in (1399, 1601, True, 1500.0, float("nan")):
            with self.subTest(p=p), self.assertRaises(ValueError):
                angle_from_p(model, p)
        for changes in (
            {"a_deg_per_p": float("nan")},
            {"a_deg_per_p": 0},
            {"b_deg": True},
            {"p_min": 1600},
            {"kind": "unknown"},
        ):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                angle_from_p({**model, **changes}, 1500)


class GripperTableTests(unittest.TestCase):
    def test_piecewise_interpolation_preserves_nonuniform_segments(self):
        table = build_gripper_table(list(reversed(width_samples())))
        self.assertEqual(table["width_direction"], "increasing")
        self.assertAlmostEqual(gripper_width_from_p(table, 1250), 17.5)
        self.assertAlmostEqual(gripper_width_from_p(table, 1750), 42.5)
        self.assertEqual(gripper_width_from_p(table, 1000), 10)
        self.assertEqual(gripper_width_from_p(table, 2000), 60)
        self.assertFalse(table["force_or_grasp_measurement"])
        self.assertFalse(table["defines_safe_motion_limits"])

    def test_decreasing_width_and_zero_width_endpoint_allowed(self):
        table = build_gripper_table(width_samples((60, 25, 0)))
        self.assertEqual(table["width_direction"], "decreasing")
        self.assertAlmostEqual(gripper_width_from_p(table, 1750), 12.5)

    def test_flat_nonmonotonic_negative_and_unverified_width_rejected(self):
        for values in (
            (5, 5, 5),
            (0, 10, 10),
            (0, 20, 10),
            (-1, 5, 10),
            (0, True, 10),
            (0, float("nan"), 10),
        ):
            with self.subTest(values=values), self.assertRaises(ValueError):
                build_gripper_table(width_samples(values))
        rows = width_samples()
        rows[0]["external_measurement"] = False
        with self.assertRaises(ValueError):
            build_gripper_table(rows)

    def test_width_extrapolation_and_tampered_table_rejected(self):
        table = build_gripper_table(width_samples())
        for p in (999, 2001, True, float("inf")):
            with self.subTest(p=p), self.assertRaises(ValueError):
                gripper_width_from_p(table, p)
        for edits in ("duplicate", "reverse", "nonmonotonic", "negative", "short"):
            changed = deepcopy(table)
            if edits == "duplicate":
                changed["points"][1]["p"] = 1000
            elif edits == "reverse":
                changed["points"].reverse()
            elif edits == "nonmonotonic":
                changed["points"][1]["width_mm"] = 80
            elif edits == "negative":
                changed["points"][0]["width_mm"] = -1
            else:
                changed["points"].pop()
            with self.subTest(edits=edits), self.assertRaises(ValueError):
                gripper_width_from_p(changed, 1500)


if __name__ == "__main__":
    unittest.main()
