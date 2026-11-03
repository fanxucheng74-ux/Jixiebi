import unittest

import cv2
import numpy as np

from arm_project.vision.colors import annotate, detect_blocks


class ColorDetectionTests(unittest.TestCase):
    def test_three_colors_have_expected_centers(self):
        frame = np.full((200, 360, 3), 160, dtype=np.uint8)
        cv2.rectangle(frame, (20, 40), (80, 100), (0, 0, 255), -1)
        cv2.rectangle(frame, (130, 40), (190, 100), (255, 0, 0), -1)
        cv2.rectangle(frame, (240, 40), (300, 100), (0, 255, 0), -1)
        detections, masks = detect_blocks(frame)
        self.assertEqual([item["color"] for item in detections], ["red", "blue", "green"])
        for item, expected_x in zip(detections, (50, 160, 270)):
            self.assertAlmostEqual(item["center_px"][0], expected_x)
            self.assertAlmostEqual(item["center_px"][1], 70)
            self.assertEqual(item["bbox_xywh"][2:], [61, 61])
        for mask in masks.values():
            self.assertEqual(mask.shape, frame.shape[:2])
            self.assertEqual(mask.dtype, np.uint8)

    def test_two_separate_red_objects_are_not_merged(self):
        frame = np.zeros((100, 180, 3), dtype=np.uint8)
        frame[20:60, 15:55] = (0, 0, 255)
        frame[20:60, 110:150] = (0, 0, 255)
        detections, _ = detect_blocks(frame)
        self.assertEqual(len(detections), 2)
        self.assertTrue(all(item["color"] == "red" for item in detections))
        self.assertLess(detections[0]["center_px"][0], detections[1]["center_px"][0])

    def test_gray_and_low_saturation_are_not_red(self):
        gray = np.full((60, 60, 3), 128, dtype=np.uint8)
        self.assertEqual(detect_blocks(gray)[0], [])
        # H 接近红色，但低饱和度不是可信的红色候选。
        hsv = np.zeros((60, 60, 3), dtype=np.uint8)
        hsv[:] = (0, 20, 160)
        frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        self.assertEqual(detect_blocks(frame)[0], [])

    def test_red_at_upper_hue_boundary(self):
        hsv = np.zeros((100, 100, 3), dtype=np.uint8)
        hsv[20:70, 20:70] = (179, 255, 255)
        frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        detections, _ = detect_blocks(frame)
        self.assertEqual(len(detections), 1)
        self.assertEqual(detections[0]["color"], "red")

    def test_opening_removes_specks_and_area_filters_small_regions(self):
        frame = np.zeros((100, 100, 3), dtype=np.uint8)
        frame[5, 5] = (0, 255, 0)
        frame[15:25, 15:25] = (255, 0, 0)
        detections, masks = detect_blocks(frame, min_area=300)
        self.assertEqual(detections, [])
        self.assertEqual(masks["green"][5, 5], 0)
        self.assertGreater(np.count_nonzero(masks["blue"]), 0)

    def test_detection_and_annotation_preserve_original(self):
        frame = np.full((100, 100, 3), 120, dtype=np.uint8)
        frame[20:70, 20:70] = (0, 0, 255)
        original = frame.copy()
        detections, _ = detect_blocks(frame)
        result = annotate(frame, detections)
        np.testing.assert_array_equal(frame, original)
        self.assertFalse(np.shares_memory(frame, result))
        self.assertFalse(np.array_equal(frame, result))

    def test_bad_input_fails_explicitly(self):
        for frame in (
            None,
            np.zeros((10, 10), dtype=np.uint8),
            np.zeros((0, 0, 3), dtype=np.uint8),
            np.zeros((10, 10, 3), dtype=np.float32),
        ):
            with self.subTest(shape=getattr(frame, "shape", None)):
                with self.assertRaises(ValueError):
                    detect_blocks(frame)
        valid = np.zeros((50, 50, 3), dtype=np.uint8)
        for min_area in (-1, 0, float("nan"), float("inf")):
            with self.subTest(min_area=min_area), self.assertRaises(ValueError):
                detect_blocks(valid, min_area)


if __name__ == "__main__":
    unittest.main()
