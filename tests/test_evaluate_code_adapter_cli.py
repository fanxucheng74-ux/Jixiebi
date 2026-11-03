"""Held-out evaluation output contracts without an ML backend or device."""

import contextlib
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from tools.evaluate_code_adapter import main


def fixture_root(directory):
    root = Path(directory)
    tools = root / "tools"
    tools.mkdir()
    for name in (
        "evaluate_code_adapter.py",
        "adapter_evaluation_inputs.py",
        "train_code_adapter.py",
        "training_examples.py",
        "code_training_precision.py",
    ):
        (tools / name).write_text('"""Fixture provenance only."""\n', encoding="utf-8")
    return root


def complete_evaluation(args, output, report, event):
    report.update(
        evaluation_completed=True,
        test_examples_evaluated=5,
        selected_with_test=False,
        base_test={"count": 5, "token_weighted_loss": 0.8},
        adapter_test={"count": 5, "token_weighted_loss": 0.7},
        comparison={"test_loss_improved": True, "functional_code_improvement_verified": False},
    )
    event("heldout_loss_evaluation_complete")


class AdapterEvaluationCliTests(unittest.TestCase):
    def test_existing_output_prevents_evaluation_or_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            root = fixture_root(directory)
            output = root / "artifacts" / "prior"
            output.mkdir(parents=True)
            marker = output / "keep.txt"
            marker.write_bytes(b"prior evaluation evidence")
            with (
                patch("tools.evaluate_code_adapter.ROOT", root),
                patch("tools.evaluate_code_adapter.run") as run,
            ):
                with self.assertRaises(FileExistsError):
                    main(["--output-dir", str(output)])
                run.assert_not_called()
            self.assertEqual(marker.read_bytes(), b"prior evaluation evidence")

    def test_output_must_be_below_artifacts_not_sources_or_dataset(self):
        with tempfile.TemporaryDirectory() as directory:
            root = fixture_root(directory)
            with (
                patch("tools.evaluate_code_adapter.ROOT", root),
                patch("tools.evaluate_code_adapter.run") as run,
            ):
                for output in (
                    root / "artifacts",
                    root / "data" / "datasets" / "frozen" / "evaluation",
                    root / "data" / "models" / "evaluation",
                    root / "tools" / "evaluation",
                ):
                    with self.subTest(output=output), self.assertRaises(ValueError):
                        main(["--output-dir", str(output)])
                run.assert_not_called()

    def test_completed_fixed_test_reports_pass_without_claiming_code_accuracy(self):
        with tempfile.TemporaryDirectory() as directory:
            root = fixture_root(directory)
            output = root / "artifacts" / "evaluation"
            printed = io.StringIO()
            with (
                patch("tools.evaluate_code_adapter.ROOT", root),
                patch("tools.evaluate_code_adapter.run", side_effect=complete_evaluation),
                contextlib.redirect_stdout(printed),
            ):
                self.assertEqual(main(["--output-dir", str(output)]), 0)
            report = json.loads((output / "report.json").read_text(encoding="utf-8"))
            self.assertEqual(report["status"], "PASS")
            self.assertEqual(report["test_examples_evaluated"], 5)
            self.assertTrue(report["evaluation_completed"])
            self.assertEqual(report["base_test"]["count"], 5)
            self.assertEqual(report["adapter_test"]["count"], 5)
            for field in (
                "training_started",
                "selected_with_test",
                "robot_io",
                "downloads_enabled",
                "generated_text_executed",
                "functional_code_improvement_verified",
                "generation_performed",
            ):
                self.assertIs(report[field], False)
            self.assertIn("EVALUATION_PASS", printed.getvalue())
            self.assertNotIn("FINETUNE_PASS", printed.getvalue())
            event = json.loads((output / "events.jsonl").read_text(encoding="utf-8"))
            self.assertEqual(event["stage"], "heldout_loss_evaluation_complete")

    def test_incomplete_or_test_selected_run_cannot_claim_pass(self):
        mutations = (
            {"evaluation_completed": False},
            {"test_examples_evaluated": 4},
            {"test_examples_evaluated": 6},
            {"selected_with_test": True},
        )
        for mutation in mutations:
            with self.subTest(mutation=mutation), tempfile.TemporaryDirectory() as directory:
                root = fixture_root(directory)
                output = root / "artifacts" / "incomplete"

                def incomplete(args, folder, report, event):
                    complete_evaluation(args, folder, report, event)
                    report.update(mutation)

                with (
                    patch("tools.evaluate_code_adapter.ROOT", root),
                    patch("tools.evaluate_code_adapter.run", side_effect=incomplete),
                    contextlib.redirect_stdout(io.StringIO()) as printed,
                ):
                    self.assertEqual(main(["--output-dir", str(output)]), 1)
                report = json.loads((output / "report.json").read_text(encoding="utf-8"))
                self.assertEqual(report["status"], "FAIL")
                self.assertIn("fixed test5", report["error"])
                self.assertNotIn("EVALUATION_PASS", printed.getvalue())

    def test_return_without_work_cannot_claim_success(self):
        with tempfile.TemporaryDirectory() as directory:
            root = fixture_root(directory)
            output = root / "artifacts" / "no_work"
            with (
                patch("tools.evaluate_code_adapter.ROOT", root),
                patch("tools.evaluate_code_adapter.run"),
                contextlib.redirect_stdout(io.StringIO()),
            ):
                self.assertEqual(main(["--output-dir", str(output)]), 1)
            report = json.loads((output / "report.json").read_text(encoding="utf-8"))
            self.assertFalse(report["evaluation_completed"])
            self.assertEqual(report["test_examples_evaluated"], 0)

    def test_failure_after_base_preserves_partial_results_and_traceback(self):
        with tempfile.TemporaryDirectory() as directory:
            root = fixture_root(directory)
            output = root / "artifacts" / "failed"

            def fail_after_base(args, folder, report, event):
                (folder / "base_test_examples.jsonl").write_text(
                    '{"id":"heldout-one","loss":0.8}\n', encoding="utf-8"
                )
                report["base_test"] = {"count": 5, "token_weighted_loss": 0.8}
                event("load_selected_adapter_for_inference")
                raise RuntimeError("synthetic adapter load failure")

            with (
                patch("tools.evaluate_code_adapter.ROOT", root),
                patch("tools.evaluate_code_adapter.run", side_effect=fail_after_base),
                contextlib.redirect_stdout(io.StringIO()),
            ):
                self.assertEqual(main(["--output-dir", str(output)]), 1)
            report = json.loads((output / "report.json").read_text(encoding="utf-8"))
            self.assertEqual(report["status"], "FAIL")
            self.assertEqual(report["last_stage"], "load_selected_adapter_for_inference")
            self.assertIn("synthetic adapter load failure", report["error_traceback"])
            self.assertIn("Traceback", report["error_traceback"])
            self.assertEqual(report["base_test"]["count"], 5)
            self.assertTrue((output / "base_test_examples.jsonl").is_file())

    def test_cuda_query_failure_does_not_hide_original_exception(self):
        with tempfile.TemporaryDirectory() as directory:
            root = fixture_root(directory)
            output = root / "artifacts" / "cuda_failure"
            with (
                patch("tools.evaluate_code_adapter.ROOT", root),
                patch(
                    "tools.evaluate_code_adapter.run", side_effect=RuntimeError("original error")
                ),
                patch.dict("sys.modules", {"torch": SimpleNamespace()}),
                patch(
                    "tools.evaluate_code_adapter.gpu_memory",
                    side_effect=RuntimeError("context lost"),
                ),
                contextlib.redirect_stdout(io.StringIO()),
            ):
                self.assertEqual(main(["--output-dir", str(output)]), 1)
            report = json.loads((output / "report.json").read_text(encoding="utf-8"))
            self.assertEqual(report["error"], "RuntimeError: original error")
            self.assertEqual(report["memory_query_error"], "RuntimeError: context lost")
            self.assertIn("original error", report["error_traceback"])


if __name__ == "__main__":
    unittest.main()
