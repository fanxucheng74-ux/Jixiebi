"""只用假相机和假writer；测试不会导入cv2或打开相机。"""

from datetime import datetime, timedelta, timezone
import json
from pathlib import Path
import shutil
import unittest
import uuid

from arm_project.cli.record_workspace import record_workspace


ROOT = Path(__file__).resolve().parents[1]


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds

    def wall_time(self):
        return (
            datetime(2026, 10, 31, tzinfo=timezone.utc) + timedelta(seconds=self.now)
        ).isoformat()


class Frame:
    shape = (480, 640, 3)


class FakeCamera:
    def __init__(self, clock, scenario):
        self.clock, self.scenario = clock, scenario
        self.released, self.reads, self.settings = False, 0, []

    def isOpened(self):
        return self.scenario != "closed"

    def set(self, key, value):
        self.settings.append((key, value))
        return True

    def read(self):
        self.clock.sleep(0.25)
        self.reads += 1
        if self.scenario == "interrupt" and self.reads == 2:
            raise KeyboardInterrupt
        return (False, None) if self.scenario == "empty" else (True, Frame())

    def release(self):
        self.released = True


class FakeWriter:
    def __init__(self, scenario):
        self.scenario, self.written, self.released = scenario, [], False

    def isOpened(self):
        return self.scenario != "writer_closed"

    def write(self, frame):
        self.written.append(frame)

    def release(self):
        self.released = True


class FakeCV:
    CAP_DSHOW, CAP_PROP_FRAME_WIDTH, CAP_PROP_FRAME_HEIGHT = 700, 3, 4

    def __init__(self, clock, scenario="normal"):
        self.scenario = scenario
        self.camera = FakeCamera(clock, scenario)
        self.writer = FakeWriter(scenario)
        self.opens = []

    def VideoCapture(self, index, backend):
        self.opens.append((index, backend))
        return self.camera

    def VideoWriter_fourcc(self, *letters):
        assert letters == tuple("MJPG")
        return 123

    def VideoWriter(self, path, fourcc, fps, size):
        self.video_settings = (path, fourcc, fps, size)
        Path(path).write_bytes(b"FAKE VIDEO FOR TEST ONLY")
        return self.writer

    def imwrite(self, path, frame):
        raise AssertionError("Unicode filesystem paths must not be passed to cv2.imwrite")

    def imencode(self, extension, frame):
        assert extension == ".png"

        class Encoded:
            def tobytes(self):
                return b"FAKE PNG FOR TEST ONLY"

        return self.scenario != "png_encode_failed", Encoded()


class RecordingTests(unittest.TestCase):
    def setUp(self):
        self.directory = ROOT / "tests" / (".camera-test-" + uuid.uuid4().hex)
        self.directory.mkdir()
        self.addCleanup(self.cleanup)

    def cleanup(self):
        target = self.directory.resolve()
        if target.parent != (ROOT / "tests").resolve() or not target.name.startswith(
            ".camera-test-"
        ):
            raise RuntimeError("unexpected test cleanup target")
        shutil.rmtree(target)

    def run_recording(self, scenario, run_id):
        clock = FakeClock()
        fake = FakeCV(clock, scenario)
        result = record_workspace(
            0, 2, run_id, project_root=self.directory, cv2_module=fake, clock=clock
        )
        return result, fake, self.directory / "runs/camera_records" / run_id

    def test_recording_uses_real_elapsed_clock_snapshots_and_ready(self):
        result, fake, directory = self.run_recording("normal", "normal")
        self.assertEqual(result["status"], "completed")
        self.assertEqual(fake.opens, [(0, fake.CAP_DSHOW)])
        self.assertEqual(fake.camera.settings, [(3, 640), (4, 480)])
        self.assertTrue(fake.camera.released and fake.writer.released)
        self.assertEqual(result["frames_written"], len(fake.writer.written))
        self.assertGreaterEqual(result["actual_elapsed_s"], 2)
        rows = [
            json.loads(line)
            for line in (directory / "frames.jsonl").read_text(encoding="utf-8").splitlines()
        ]
        self.assertEqual(len(rows), result["frames_written"])
        self.assertEqual([r["frame_number"] for r in rows], list(range(1, len(rows) + 1)))
        self.assertEqual(
            result["actual_sampling_span_s"], rows[-1]["monotonic_s"] - rows[0]["monotonic_s"]
        )
        png_rows = [r for r in rows if r["png_path"]]
        self.assertEqual(len(png_rows), 2)
        self.assertGreaterEqual(png_rows[1]["monotonic_s"] - png_rows[0]["monotonic_s"], 1)
        ready = json.loads((directory / "ready.json").read_text(encoding="utf-8"))
        self.assertTrue(ready["ready"] and ready["first_frame_submitted"])
        self.assertTrue(Path(ready["first_frame"]).exists())

    def test_camera_writer_or_no_frames_fail_without_ready_and_release_resources(self):
        for scenario in ("closed", "writer_closed", "empty", "png_encode_failed"):
            with self.subTest(scenario=scenario):
                result, fake, directory = self.run_recording(scenario, scenario)
                self.assertEqual(result["status"], "failed")
                self.assertFalse((directory / "ready.json").exists())
                self.assertTrue(fake.camera.released)
                self.assertTrue((directory / "result.json").exists())
                if scenario == "writer_closed":
                    self.assertTrue(fake.writer.released)

    def test_unicode_project_root_saves_encoded_png_and_preserves_original_bytes(self):
        clock = FakeClock()
        fake = FakeCV(clock)
        result = record_workspace(
            0,
            2,
            "unicode",
            project_root=self.directory / "机械臂项目",
            cv2_module=fake,
            clock=clock,
        )
        self.assertEqual(result["status"], "completed")
        directory = self.directory / "机械臂项目/runs/camera_records/unicode"
        ready = json.loads((directory / "ready.json").read_text(encoding="utf-8"))
        self.assertEqual(Path(ready["first_frame"]).read_bytes(), b"FAKE PNG FOR TEST ONLY")
        self.assertTrue(fake.camera.released and fake.writer.released)

    def test_interrupt_after_ready_invalidates_ready_and_releases_both(self):
        result, fake, directory = self.run_recording("interrupt", "interrupted")
        self.assertEqual(result["status"], "failed")
        self.assertTrue(result["interrupted"])
        self.assertEqual(result["frames_written"], 1)
        self.assertFalse((directory / "ready.json").exists())
        self.assertTrue(fake.camera.released and fake.writer.released)

    def test_bad_arguments_and_existing_run_do_not_open_camera(self):
        clock = FakeClock()
        fake = FakeCV(clock)
        for seconds, run_id in (
            (1, "x"),
            (61, "x"),
            (float("nan"), "x"),
            (True, "x"),
            (2, "../escape"),
            (2, "x/y"),
        ):
            with self.subTest(seconds=seconds, run_id=run_id), self.assertRaises(ValueError):
                record_workspace(
                    0, seconds, run_id, project_root=self.directory, cv2_module=fake, clock=clock
                )
        directory = self.directory / "runs/camera_records/existing"
        directory.mkdir(parents=True)
        with self.assertRaises(FileExistsError):
            record_workspace(
                0, 2, "existing", project_root=self.directory, cv2_module=fake, clock=clock
            )
        self.assertEqual(fake.opens, [])


if __name__ == "__main__":
    unittest.main()
