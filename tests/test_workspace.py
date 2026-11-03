"""Synthetic image/plane fixtures only; never authorize or connect to hardware."""

from contextlib import redirect_stdout
from copy import deepcopy
import hashlib
import io
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch
from uuid import uuid4

import numpy as np

from arm_project.cli.select_target import main
from arm_project.vision.files import write_png
from arm_project.vision.workspace import PlanarMap, select_target


SIZE = [200, 200]
ROI = [[20, 20], [180, 20], [180, 180], [20, 180]]
# Invented mathematical oracle, never a real camera or arm measurement.
H = np.array([[0.0, -0.5, 100], [0.5, 0, -10], [0.0005, 0.0002, 1]])


def mapped(point):
    p = H @ [*point, 1]
    return (p[:2] / p[2]).tolist()


def mapping():
    def pairs(points):
        return [{"pixel_xy": list(p), "plane_xy_mm": mapped(p)} for p in points]

    return {
        "source": "SYNTHETIC",
        "plane_frame": "TEST_PLANE_NOT_REAL_ARM",
        "pixel_space": "RAW_PIXELS",
        "calibration": pairs(ROI),
        "validation": pairs([[60, 60], [140, 70], [80, 140]]),
        "max_error_mm": 0.001,
    }


def configuration(calibrated=False):
    return {
        "schema_version": 1,
        "image_size_wh": SIZE.copy(),
        "pixel_space": "RAW_PIXELS",
        "roi_polygon_px": deepcopy(ROI),
        "min_area_px2": 50,
        "max_area_px2": 5000,
        "max_aspect_ratio": 3,
        "plane_mapping": mapping() if calibrated else None,
    }


def image():
    return np.full((200, 200, 3), 128, dtype=np.uint8)


class PlaneMappingTests(unittest.TestCase):
    def test_perspective_mapping_matches_independent_mathematical_oracle(self):
        model = PlanarMap(mapping(), SIZE, "RAW_PIXELS")
        for point in [[30, 100], [100, 100], [160, 150]]:
            np.testing.assert_allclose(model.transform(point), mapped(point), atol=1e-5)
        self.assertLess(model.metrics["validation_max_error_mm"], 0.001)

    def test_outside_support_is_rejected_not_extrapolated(self):
        model = PlanarMap(mapping(), SIZE, "RAW_PIXELS")
        for point in [[19, 100], [190, 100], [100, -1]]:
            with self.subTest(point=point), self.assertRaises(ValueError):
                model.transform(point)

    def test_invalid_collinear_duplicate_and_infinite_correspondences_rejected(self):
        bad = []
        spec = mapping()
        spec["calibration"] = spec["calibration"][:3]
        bad.append(spec)
        spec = mapping()
        spec["calibration"][1] = deepcopy(spec["calibration"][0])
        bad.append(spec)
        spec = mapping()
        for i, row in enumerate(spec["calibration"]):
            row["pixel_xy"] = [30 + i * 20, 30 + i * 20]
        bad.append(spec)
        spec = mapping()
        spec["calibration"][0]["plane_xy_mm"][0] = float("nan")
        bad.append(spec)
        spec = mapping()
        spec["calibration"][0]["pixel_xy"][0] = 10**1000
        bad.append(spec)
        for spec in bad:
            with self.subTest(spec=str(spec)[:60]), self.assertRaises(ValueError):
                PlanarMap(spec, SIZE, "RAW_PIXELS")

    def test_bad_held_out_measurements_fail_even_when_fit_is_exact(self):
        spec = mapping()
        spec["validation"][0]["plane_xy_mm"][0] += 10
        with self.assertRaisesRegex(ValueError, "error limit"):
            PlanarMap(spec, SIZE, "RAW_PIXELS")

    def test_validation_must_be_independent_and_supported(self):
        for point in [[20, 20], [190, 100]]:
            spec = mapping()
            spec["validation"][0] = {"pixel_xy": point, "plane_xy_mm": mapped(point)}
            with self.subTest(point=point), self.assertRaises(ValueError):
                PlanarMap(spec, SIZE, "RAW_PIXELS")

    def test_explicit_source_frame_pixel_space_and_tolerance_required(self):
        for key, value in (
            ("source", "UNKNOWN"),
            ("plane_frame", ""),
            ("pixel_space", "UNDISTORTED_PIXELS"),
            ("max_error_mm", True),
            ("max_error_mm", 0),
            ("max_error_mm", float("inf")),
        ):
            spec = mapping()
            spec[key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                PlanarMap(spec, SIZE, "RAW_PIXELS")


class WorkspaceTests(unittest.TestCase):
    def test_unique_requested_color_selected_pixels_only(self):
        frame = image()
        frame[50:80, 50:80] = (0, 0, 255)
        frame[100:130, 100:130] = (255, 0, 0)
        result = select_target(frame, configuration(), "red")
        self.assertEqual(result["status"], "TARGET_CANDIDATE")
        self.assertEqual(result["selected"]["center_px"], [64.5, 64.5])
        self.assertIsNone(result["selected"]["plane_xy_mm"])
        self.assertTrue(result["needs_plane_calibration"])
        self.assertFalse(result["physical_accuracy_verified"])
        self.assertFalse(result["motion_authorized"])

    def test_calibrated_target_has_expected_xy_but_no_motion_permission(self):
        frame = image()
        frame[50:80, 50:80] = (0, 0, 255)
        result = select_target(frame, configuration(True), "red")
        np.testing.assert_allclose(
            result["selected"]["plane_xy_mm"], mapped([64.5, 64.5]), atol=1e-5
        )
        self.assertEqual(result["mapping_source"], "SYNTHETIC")
        self.assertFalse(result["motion_authorized"])

    def test_background_and_partially_cropped_object_are_excluded(self):
        for bounds in [(slice(2, 17), slice(2, 17)), (slice(10, 50), slice(30, 70))]:
            frame = image()
            frame[bounds] = (0, 255, 0)
            result = select_target(frame, configuration(), "green")
            self.assertEqual(result["status"], "NO_TARGET")
            self.assertIsNone(result["selected"])
            self.assertEqual(result["rejected"][0]["reason"], "OUTSIDE_WORKSPACE_OR_PARTIAL_OBJECT")

    def test_two_targets_never_selects_largest_or_first_implicitly(self):
        frame = image()
        frame[40:70, 40:70] = (0, 0, 255)
        frame[100:140, 100:140] = (0, 0, 255)
        result = select_target(frame, configuration(), "red")
        self.assertEqual(result["status"], "AMBIGUOUS")
        self.assertEqual(result["candidate_count"], 2)
        self.assertIsNone(result["selected"])

    def test_object_cut_by_image_edge_is_rejected_even_with_full_image_roi(self):
        spec = configuration()
        spec["roi_polygon_px"] = [[0, 0], [199, 0], [199, 199], [0, 199]]
        frame = image()
        frame[0:40, 30:70] = (0, 0, 255)
        result = select_target(frame, spec, "red")
        self.assertEqual(result["status"], "NO_TARGET")
        self.assertEqual(result["rejected"][0]["reason"], "IMAGE_EDGE_OBJECT")

    def test_shape_and_maximum_area_limits(self):
        for bounds, reason in [
            ((slice(30, 40), slice(30, 140)), "ASPECT_RATIO"),
            ((slice(30, 130), slice(30, 130)), "AREA_TOO_LARGE"),
        ]:
            frame = image()
            frame[bounds] = (255, 0, 0)
            result = select_target(frame, configuration(), "blue")
            self.assertIsNone(result["selected"])
            self.assertEqual(result["rejected"][0]["reason"], reason)

    def test_wrong_resolution_invalid_roi_and_missing_configuration_fail_closed(self):
        for key, value in (
            ("schema_version", True),
            ("image_size_wh", [100, 100]),
            ("roi_polygon_px", []),
            ("roi_polygon_px", [[0, 0], [5, 5], [10, 10]]),
            ("roi_polygon_px", [[20, 20], [180, 180], [180, 20], [20, 180]]),
            ("min_area_px2", True),
            ("max_aspect_ratio", 0),
        ):
            spec = configuration()
            spec[key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                select_target(image(), spec, "red")

    def test_detection_and_mapping_do_not_mutate_image_or_config(self):
        frame, spec = image(), configuration(True)
        frame[50:80, 50:80] = (0, 0, 255)
        original, original_spec = frame.copy(), deepcopy(spec)
        select_target(frame, spec, "red")
        np.testing.assert_array_equal(frame, original)
        self.assertEqual(spec, original_spec)

    def test_bbox_outside_calibration_support_is_not_mapped(self):
        spec = configuration(True)
        spec["roi_polygon_px"] = [[0, 0], [199, 0], [199, 199], [0, 199]]
        frame = image()
        frame[40:70, 10:40] = (0, 0, 255)
        result = select_target(frame, spec, "red")
        self.assertEqual(result["status"], "NO_TARGET")
        self.assertEqual(result["rejected"][0]["reason"], "OUTSIDE_CALIBRATION_HULL")


class WorkspaceCliTests(unittest.TestCase):
    def setUp(self):
        self.parent = Path(tempfile.gettempdir()).resolve()
        self.root = self.parent / ("arm_workspace_test_" + uuid4().hex)
        self.root.mkdir()
        self.addCleanup(self.cleanup)
        frame = image()
        frame[50:80, 50:80] = (0, 0, 255)
        self.image = self.root / "原图.png"
        self.config = self.root / "workspace.json"
        write_png(self.image, frame)
        self.config.write_text(json.dumps(configuration(True)), encoding="utf-8")

    def cleanup(self):
        resolved = self.root.resolve()
        if resolved.parent != self.parent or not resolved.name.startswith("arm_workspace_test_"):
            raise AssertionError("cleanup path mismatch")
        shutil.rmtree(resolved)

    def run_cli(self, color="red"):
        args = [
            "--image",
            str(self.image),
            "--config",
            str(self.config),
            "--color",
            color,
            "--output",
            str(self.root / "result"),
        ]
        with (
            patch("cv2.VideoCapture", side_effect=AssertionError("no camera")),
            patch(
                "arm_project.unified_arm._RawSerialTransport.open",
                side_effect=AssertionError("no serial"),
            ),
            redirect_stdout(io.StringIO()),
        ):
            return main(args)

    def test_cli_saves_candidate_and_preserves_inputs_and_refuses_overwrite(self):
        old = hashlib.sha256(self.image.read_bytes() + self.config.read_bytes()).hexdigest()
        self.assertEqual(self.run_cli(), 0)
        result = json.loads((self.root / "result/result.json").read_text(encoding="utf-8"))
        self.assertFalse(result["motion_authorized"])
        self.assertEqual(result["mapping_source"], "SYNTHETIC")
        self.assertEqual(self.run_cli(), 1)
        self.assertEqual(
            hashlib.sha256(self.image.read_bytes() + self.config.read_bytes()).hexdigest(), old
        )

    def test_no_target_records_diagnosis_and_returns_nonzero(self):
        self.assertEqual(self.run_cli("blue"), 2)
        result = json.loads((self.root / "result/result.json").read_text(encoding="utf-8"))
        self.assertEqual(result["status"], "NO_TARGET")
        self.assertIsNone(result["selected"])

    def test_empty_roi_is_rejected_before_creating_output(self):
        spec = configuration()
        spec["roi_polygon_px"] = []
        self.config.write_text(json.dumps(spec), encoding="utf-8")
        self.assertEqual(self.run_cli(), 1)
        self.assertFalse((self.root / "result").exists())


if __name__ == "__main__":
    unittest.main()
