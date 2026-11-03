"""无硬件测试 RK 无窗口相机采集入口。"""

import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from arm_project.cli.capture_rk import capture


class FakeCamera:
    def __init__(self, *, opened=True, fail_at=None):
        self.opened = opened
        self.fail_at = fail_at
        self.reads = 0
        self.released = False
        self.settings = []

    def isOpened(self):
        return self.opened

    def set(self, key, value):
        self.settings.append((key, value))
        return True

    def read(self):
        self.reads += 1
        if self.reads == self.fail_at:
            return False, None
        return True, np.zeros((24, 32, 3), dtype=np.uint8)

    def release(self):
        self.released = True


class CaptureRKTests(unittest.TestCase):
    def test_saves_actual_dimensions_and_releases_camera(self):
        camera = FakeCamera()
        calls = []

        def factory(device, backend):
            calls.append((device, backend))
            return camera

        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp) / "run"
            result = capture("/dev/video7", folder, warmup=2, camera_factory=factory)
            self.assertEqual(calls, [("/dev/video7", cv2.CAP_V4L2)])
            self.assertEqual(camera.reads, 3)
            self.assertTrue(camera.released)
            self.assertEqual(result["actual_size_wh"], [32, 24])
            self.assertEqual(result["requested_size_wh"], [640, 480])
            self.assertFalse(result["robot_motion_authorized"])
            self.assertTrue((folder / "frame.png").is_file())
            self.assertEqual(
                json.loads((folder / "capture.json").read_text())["device"], "/dev/video7"
            )
            with self.assertRaises(FileExistsError):
                capture("/dev/video7", folder, camera_factory=factory)

    def test_read_failure_releases_camera_without_image(self):
        camera = FakeCamera(fail_at=1)
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp) / "run"
            with self.assertRaisesRegex(RuntimeError, "warmup"):
                capture("/dev/video7", folder, camera_factory=lambda *args: camera)
            self.assertTrue(camera.released)
            self.assertFalse((folder / "frame.png").exists())

    def test_invalid_device_rejected_without_opening_camera(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(ValueError):
                capture("0", Path(tmp) / "run", camera_factory=lambda *_: self.fail("opened"))

    def test_closed_device_released(self):
        camera = FakeCamera(opened=False)
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaisesRegex(RuntimeError, "cannot open"):
                capture("/dev/video7", Path(tmp) / "run", camera_factory=lambda *_: camera)
        self.assertTrue(camera.released)


if __name__ == "__main__":
    unittest.main()
