"""Finite training CLI tests simulate failures; no ML libraries or GPU required."""

import contextlib
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from tools.train_code_adapter import main


class FormalTrainingCliTests(unittest.TestCase):
    def test_output_exists_before_any_training_and_cannot_be_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            (output / "keep").write_bytes(b"prior adapter evidence")
            with patch("tools.train_code_adapter.run") as run:
                with self.assertRaises(FileExistsError):
                    main(["--output-dir", str(output)])
                run.assert_not_called()
            self.assertEqual((output / "keep").read_bytes(), b"prior adapter evidence")

    def test_invalid_settings_and_source_outputs_fail_before_model_work(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "unused"
            with patch("tools.train_code_adapter.run") as run:
                for option, value in (
                    ("--epochs", "0"),
                    ("--epochs", "6"),
                    ("--learning-rate", "nan"),
                    ("--accumulation-steps", "0"),
                    ("--diagnostic-steps", "-1"),
                    ("--diagnostic-steps", "3"),
                ):
                    with self.assertRaises(ValueError):
                        main(["--output-dir", str(output), option, value])
                with self.assertRaises(ValueError):
                    main(["--output-dir", str(output), "--dataset", str(Path(directory))])
                run.assert_not_called()
            self.assertFalse(output.exists())

    def test_failure_during_validation_preserves_saved_epoch(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "run"

            def fail_after_save(args, folder, report, event):
                checkpoint = folder / "epoch_01"
                checkpoint.mkdir()
                (checkpoint / "checkpoint_complete.json").write_text('{"saved":true}')
                report["saved_checkpoints"].append(str(checkpoint))
                report["completed_optimizer_steps"] = 10
                event("epoch_validation")
                raise RuntimeError("synthetic validation failure")

            with patch("tools.train_code_adapter.run", side_effect=fail_after_save):
                with contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(main(["--output-dir", str(output)]), 1)
            result = json.loads((output / "report.json").read_text())
            self.assertEqual(result["status"], "FAIL")
            self.assertEqual(result["completed_optimizer_steps"], 10)
            self.assertEqual(len(result["saved_checkpoints"]), 1)
            self.assertFalse(result["formal_pilot_training_completed"])
            self.assertTrue((output / "epoch_01/checkpoint_complete.json").is_file())

    def test_incomplete_run_cannot_claim_pass(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "run"
            with patch("tools.train_code_adapter.run"), contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main(["--output-dir", str(output)]), 1)
            result = json.loads((output / "report.json").read_text())
            self.assertEqual(result["status"], "FAIL")
            self.assertIn("without completing", result["error"])

    def test_failure_keeps_traceback_even_if_cuda_memory_query_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "run"
            with (
                patch("tools.train_code_adapter.run", side_effect=RuntimeError("kernel failed")),
                patch.dict("sys.modules", {"torch": SimpleNamespace()}),
                patch("tools.train_code_adapter.gpu_memory", side_effect=RuntimeError("reset")),
                contextlib.redirect_stdout(io.StringIO()),
            ):
                self.assertEqual(main(["--output-dir", str(output)]), 1)
            result = json.loads((output / "report.json").read_text())
            self.assertEqual(result["error"], "RuntimeError: kernel failed")
            self.assertEqual(result["memory_query_error"], "RuntimeError: reset")
            self.assertTrue(result["error_time"])
            self.assertIn("Traceback", result["error_traceback"])
            self.assertIn("RuntimeError: kernel failed", result["error_traceback"])
            self.assertEqual(
                (output / "failure_traceback.txt").read_text(), result["error_traceback"]
            )

    def test_diagnostic_success_is_distinct_from_formal_training(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "run"

            def finish_diagnostic(args, folder, report, event):
                report["completed_optimizer_steps"] = args.diagnostic_steps
                report["diagnostic_completed"] = True

            stdout = io.StringIO()
            with (
                patch("tools.train_code_adapter.run", side_effect=finish_diagnostic),
                contextlib.redirect_stdout(stdout),
            ):
                self.assertEqual(main(["--output-dir", str(output), "--diagnostic-steps", "2"]), 0)
            result = json.loads((output / "report.json").read_text())
            self.assertEqual(result["status"], "DIAGNOSTIC_PASS")
            self.assertFalse(result["formal_pilot_training_completed"])
            self.assertEqual(result["saved_checkpoints"], [])
            self.assertIn("DIAGNOSTIC_PASS", stdout.getvalue())
            self.assertNotIn("FINETUNE_PASS", stdout.getvalue())

    def test_invalid_diagnostic_completion_cannot_claim_success(self):
        mutations = (
            {"diagnostic_completed": False},
            {"completed_optimizer_steps": 1},
            {"completed_optimizer_steps": 3},
            {"formal_pilot_training_completed": True},
            {"saved_checkpoints": ["unexpected_adapter"]},
        )
        for mutation in mutations:
            with self.subTest(mutation=mutation), tempfile.TemporaryDirectory() as directory:
                output = Path(directory) / "run"

                def invalid_finish(args, folder, report, event):
                    report.update(diagnostic_completed=True, completed_optimizer_steps=2)
                    report.update(mutation)

                with (
                    patch("tools.train_code_adapter.run", side_effect=invalid_finish),
                    contextlib.redirect_stdout(io.StringIO()),
                ):
                    self.assertEqual(
                        main(["--output-dir", str(output), "--diagnostic-steps", "2"]), 1
                    )
                result = json.loads((output / "report.json").read_text())
                self.assertEqual(result["status"], "FAIL")
                self.assertIn("finite budget", result["error"])

    def test_default_formal_success_still_requires_formal_completion(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "run"

            def finish_formal(args, folder, report, event):
                self.assertEqual(args.diagnostic_steps, 0)
                report["formal_pilot_training_completed"] = True
                report["selection"] = {"best_checkpoint": "epoch_01"}

            stdout = io.StringIO()
            with (
                patch("tools.train_code_adapter.run", side_effect=finish_formal),
                contextlib.redirect_stdout(stdout),
            ):
                self.assertEqual(main(["--output-dir", str(output)]), 0)
            result = json.loads((output / "report.json").read_text())
            self.assertEqual(result["status"], "PASS")
            self.assertIn("FINETUNE_PASS", stdout.getvalue())

    def test_inherited_synchronous_debug_flag_is_recorded_in_formal_mode(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "run"
            with (
                patch.dict("os.environ", {"CUDA_LAUNCH_BLOCKING": "1"}),
                patch("tools.train_code_adapter.run", side_effect=RuntimeError("no GPU used")),
                contextlib.redirect_stdout(io.StringIO()),
            ):
                self.assertEqual(main(["--output-dir", str(output)]), 1)
            result = json.loads((output / "report.json").read_text())
            self.assertEqual(result["mode"], "formal")
            self.assertTrue(result["config"]["cuda_launch_blocking"])


if __name__ == "__main__":
    unittest.main()
