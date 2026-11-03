"""Preflight output/failure containment tests without any ML backend or device."""

import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from tools.run_code_training_preflight import main


class PreflightCliTests(unittest.TestCase):
    def test_existing_output_prevents_any_model_or_training_work(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            (folder / "keep.json").write_text("prior evidence")
            with patch("tools.run_code_training_preflight.run") as run:
                with self.assertRaises(FileExistsError):
                    main(["--output-dir", str(folder)])
                run.assert_not_called()
            self.assertEqual((folder / "keep.json").read_text(), "prior evidence")

    def test_failure_after_baseline_preserves_baseline_and_failure_stage(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory) / "new"

            def fail_after_baseline(args, output, report, event):
                (output / "baseline.json").write_text('{"completed":true}')
                report["baseline_completed"] = True
                event("longest_train_example_forward")
                raise RuntimeError("synthetic failure; never used a GPU")

            with patch("tools.run_code_training_preflight.run", side_effect=fail_after_baseline):
                with contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(main(["--output-dir", str(folder)]), 1)
            report = json.loads((folder / "report.json").read_text())
            self.assertEqual(report["status"], "FAIL")
            self.assertTrue(report["baseline_completed"])
            self.assertEqual(report["last_stage"], "longest_train_example_forward")
            self.assertEqual(report["completed_optimizer_steps"], 0)
            self.assertFalse(report["adapter_saved"])
            self.assertEqual((folder / "baseline.json").read_text(), '{"completed":true}')

    def test_reports_cannot_be_written_inside_dataset_or_model(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            with patch("tools.run_code_training_preflight.run") as run:
                for flag in ("--dataset", "--model-dir"):
                    with self.assertRaises(ValueError):
                        main([flag, str(base), "--output-dir", str(base / "child")])
                run.assert_not_called()
            self.assertFalse((base / "child").exists())


if __name__ == "__main__":
    unittest.main()
