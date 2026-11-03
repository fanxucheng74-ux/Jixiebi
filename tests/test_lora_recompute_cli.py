"""Synthetic recomputation probe contracts without ML imports or GPU use."""

import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from tools.check_lora_recompute import LENGTHS, load_spec, main


class LoraRecomputeCliTests(unittest.TestCase):
    def test_spec_derives_gqa_projection_and_rank_from_local_json(self):
        with tempfile.TemporaryDirectory() as folder:
            config, failure = Path(folder) / "config.json", Path(folder) / "failure.json"
            config.write_text(
                json.dumps(
                    {
                        "model_type": "qwen2",
                        "hidden_size": 3584,
                        "num_attention_heads": 28,
                        "num_key_value_heads": 4,
                    }
                )
            )
            failure.write_text(json.dumps({"config": {"lora_rank": 8, "lora_alpha": 16}}))
            result = load_spec(config, failure)
            self.assertEqual(
                (
                    result["hidden_size"],
                    result["head_dim"],
                    result["v_proj_out_features"],
                    result["rank"],
                ),
                (3584, 128, 512, 8),
            )
            self.assertEqual(result["scaling"], 2)
            self.assertEqual(len(result["config_sha256"]), 64)
            config.write_text(
                json.dumps(
                    {
                        "model_type": "qwen2",
                        "hidden_size": True,
                        "num_attention_heads": 28,
                        "num_key_value_heads": 4,
                    }
                )
            )
            with self.assertRaises(ValueError):
                load_spec(config, failure)

    def test_existing_output_never_runs_probe(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "report.json"
            output.write_bytes(b"original")
            with patch("tools.check_lora_recompute.run_probe") as run:
                with self.assertRaises(FileExistsError):
                    main(["--output", str(output)])
                run.assert_not_called()
            self.assertEqual(output.read_bytes(), b"original")

    def test_invalid_repeat_budget_fails_before_output_or_probe(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "report.json"
            with patch("tools.check_lora_recompute.run_probe") as run:
                for repeat in ("0", "4", "-1"):
                    with self.assertRaises(ValueError):
                        main(["--output", str(output), "--repeats", repeat])
                run.assert_not_called()
            self.assertFalse(output.exists())

    def test_mocked_success_records_finite_plan_and_limits_claims(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "report.json"

            def complete(args, report, event):
                report["completed_optimizer_steps"] = args.repeats
                event("synthetic_complete")

            with (
                patch("tools.check_lora_recompute.run_probe", side_effect=complete),
                contextlib.redirect_stdout(io.StringIO()) as printed,
            ):
                self.assertEqual(main(["--output", str(output)]), 0)
            report = json.loads(output.read_text())
            self.assertEqual(report["sequence_lengths"], list(LENGTHS))
            self.assertEqual(report["planned_iterations"], 8)
            self.assertEqual(report["completed_optimizer_steps"], 2)
            self.assertFalse(report["full_7b_training_stability_verified"])
            self.assertFalse(report["nf4_base_loaded"])
            self.assertFalse(report["adapter_saved"])
            self.assertIn("LORA_RECOMPUTE_PASS", printed.getvalue())

    def test_failure_traceback_and_last_stage_are_preserved(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "report.json"

            def fail(args, report, event):
                event("backward", repeat=1, sequence_tokens=982)
                raise RuntimeError("synthetic GEMM failure")

            with (
                patch("tools.check_lora_recompute.run_probe", side_effect=fail),
                contextlib.redirect_stdout(io.StringIO()),
            ):
                self.assertEqual(main(["--output", str(output)]), 1)
            report = json.loads(output.read_text())
            self.assertEqual(report["status"], "FAIL")
            self.assertEqual(report["last_stage_details"]["sequence_tokens"], 982)
            self.assertIn("synthetic GEMM failure", report["error_traceback"])
            self.assertIn("Traceback", report["error_traceback"])

    def test_incomplete_mock_run_cannot_claim_success(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "report.json"
            with (
                patch("tools.check_lora_recompute.run_probe"),
                contextlib.redirect_stdout(io.StringIO()),
            ):
                self.assertEqual(main(["--output", str(output)]), 1)
            self.assertEqual(json.loads(output.read_text())["status"], "FAIL")


if __name__ == "__main__":
    unittest.main()
