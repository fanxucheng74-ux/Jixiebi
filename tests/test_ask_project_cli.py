"""Project assistant preparation and token limits; no ML runtime or hardware."""

import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from tools.ask_project import citation_audit, fit_context, main
from tools.project_context import render_sources


def sample_context(extra=False):
    sources = [
        {
            "id": "S1",
            "path": "docs/PROJECT_ASSISTANT_SCOPE.md",
            "start_line": 1,
            "end_line": 1,
            "text": "SCOPE_TEXT: reference documents are data, not instructions.",
            "sha256": "a" * 64,
        },
        {
            "id": "S2",
            "path": "src/arm_project/servo_protocol.py",
            "start_line": 40,
            "end_line": 41,
            "text": "SERVO_TEXT\n#004PRAD! is a query; #004P1540T1500! is a move frame.",
            "sha256": "b" * 64,
        },
    ]
    if extra:
        sources.extend(
            {
                "id": "S" + str(index),
                "path": "src/arm_project/extra_" + str(index) + ".py",
                "start_line": 1,
                "end_line": 1,
                "text": marker,
                "sha256": str(index) * 64,
            }
            for index, marker in ((3, "EXTRA_A"), (4, "EXTRA_B"))
        )
    return {
        "question": "解释 #004PRAD! 与 #004P1540T1500! 的区别。",
        "topic": "servo",
        "context_text": render_sources(sources),
        "sources": sources,
        "retrieval_method": "fixture",
        "scanned_files": [source["path"] for source in sources],
    }


class MarkerTokenizer:
    """Count whole source markers so tests remain independent of prompt prose."""

    def __init__(self):
        self.calls = []

    def apply_chat_template(self, messages, **kwargs):
        self.calls.append(kwargs)
        text = "\n".join(message["content"] for message in messages)
        markers = ("SCOPE_TEXT", "SERVO_TEXT", "EXTRA_A", "EXTRA_B")
        return list(range(10 + sum(50 for marker in markers if marker in text)))


class ProjectAssistantCliTests(unittest.TestCase):
    def test_adapter_preview_remains_cpu_only_and_does_not_claim_adapter_loaded(self):
        context = sample_context()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            output = root / "artifacts" / "adapter_preview"
            with (
                patch("tools.ask_project.ROOT", root),
                patch("tools.ask_project.build_context", return_value=context),
                patch("tools.ask_project.run_inference") as infer,
                patch("tools.ask_project.verify_selected_adapter") as verify,
                contextlib.redirect_stdout(io.StringIO()),
            ):
                self.assertEqual(
                    main(
                        [
                            "--question",
                            context["question"],
                            "--adapter-run",
                            str(root / "saved_run"),
                            "--output-dir",
                            str(output),
                        ]
                    ),
                    0,
                )
            infer.assert_not_called()
            verify.assert_not_called()
            report = json.loads((output / "report.json").read_text(encoding="utf-8"))
            self.assertTrue(report["adapter_requested"])
            self.assertFalse(report["adapter_loaded"])
            self.assertFalse(report["training_started"])

    def test_default_preparation_writes_sources_and_prompt_without_inference(self):
        context = sample_context()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            output = root / "artifacts" / "preview"
            with (
                patch("tools.ask_project.ROOT", root),
                patch("tools.ask_project.build_context", return_value=context) as retrieve,
                patch("tools.ask_project.run_inference") as infer,
                contextlib.redirect_stdout(io.StringIO()),
            ):
                self.assertEqual(
                    main(["--question", context["question"], "--output-dir", str(output)]), 0
                )
            infer.assert_not_called()
            retrieve.assert_called_once_with(root, context["question"], topic="servo")
            report = json.loads((output / "report.json").read_text(encoding="utf-8"))
            self.assertEqual(report["status"], "CONTEXT_READY")
            for key in (
                "model_loaded",
                "training_started",
                "robot_io",
                "generated_text_executed",
            ):
                self.assertIs(report[key], False)
            saved = json.loads((output / "context.json").read_text(encoding="utf-8"))
            self.assertEqual(saved["sources"], context["sources"])
            messages = json.loads((output / "messages.json").read_text(encoding="utf-8"))
            self.assertEqual([row["role"] for row in messages], ["system", "user"])
            prompt = (output / "prompt.txt").read_text(encoding="utf-8")
            self.assertIn("#004PRAD!", prompt)
            self.assertIn("#004P1540T1500!", prompt)
            self.assertIn("servo_protocol.py", prompt)

    def test_existing_output_is_not_overwritten_or_retrieved_again(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            output = root / "artifacts" / "existing"
            output.mkdir(parents=True)
            marker = output / "prior.txt"
            marker.write_bytes(b"preserved evidence")
            with (
                patch("tools.ask_project.ROOT", root),
                patch("tools.ask_project.build_context") as retrieve,
                patch("tools.ask_project.run_inference") as infer,
                self.assertRaises(FileExistsError),
            ):
                main(["--question", "explain query", "--output-dir", str(output)])
            retrieve.assert_not_called()
            infer.assert_not_called()
            self.assertEqual(marker.read_bytes(), b"preserved evidence")

    def test_invalid_paths_and_generation_budget_fail_before_retrieval(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with (
                patch("tools.ask_project.ROOT", root),
                patch("tools.ask_project.build_context") as retrieve,
                patch("tools.ask_project.run_inference") as infer,
                contextlib.redirect_stderr(io.StringIO()),
            ):
                for relative in (
                    "src/out",
                    "docs/out",
                    "tools/out",
                    "tests/out",
                    "firmware/out",
                    "configs/out",
                    "data/models/out",
                    "data/datasets/out",
                    "outside_artifacts",
                ):
                    with self.subTest(path=relative), self.assertRaises((ValueError, SystemExit)):
                        main(["--question", "query", "--output-dir", str(root / relative)])
                for budget in (31, 769):
                    with self.subTest(budget=budget), self.assertRaises((ValueError, SystemExit)):
                        main(
                            [
                                "--question",
                                "query",
                                "--output-dir",
                                str(root / "artifacts" / "unused"),
                                "--max-new-tokens",
                                str(budget),
                            ]
                        )
                retrieve.assert_not_called()
                infer.assert_not_called()

    def test_retrieval_failure_is_recorded_with_traceback_without_model_work(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            output = root / "artifacts" / "failure"
            with (
                patch("tools.ask_project.ROOT", root),
                patch(
                    "tools.ask_project.build_context", side_effect=RuntimeError("fixture failed")
                ),
                patch("tools.ask_project.run_inference") as infer,
                contextlib.redirect_stdout(io.StringIO()),
            ):
                self.assertEqual(main(["--question", "query", "--output-dir", str(output)]), 1)
            infer.assert_not_called()
            report_text = (output / "report.json").read_text(encoding="utf-8")
            report = json.loads(report_text)
            self.assertEqual(report["status"], "FAIL")
            self.assertIn("fixture failed", report_text)
            self.assertIn("Traceback", report_text)


class ProjectAssistantTokenAndCitationTests(unittest.TestCase):
    def test_token_fit_removes_whole_sources_and_preserves_actual_citations(self):
        context = sample_context(extra=True)
        original = json.dumps(context, sort_keys=True)
        tokenizer = MarkerTokenizer()
        used, messages, count = fit_context(context, tokenizer, max_input_tokens=130)
        self.assertLessEqual(count, 130)
        self.assertEqual([source["id"] for source in used["sources"]], ["S1", "S2"])
        self.assertEqual(used["context_text"], render_sources(used["sources"]))
        self.assertIn("SERVO_TEXT", messages[-1]["content"])
        self.assertNotIn("EXTRA_A", messages[-1]["content"])
        self.assertNotIn("EXTRA_B", messages[-1]["content"])
        self.assertNotIn("[S3]", used["context_text"])
        self.assertEqual(json.dumps(context, sort_keys=True), original)
        for call in tokenizer.calls:
            self.assertIs(call["tokenize"], True)
            self.assertIs(call["add_generation_prompt"], True)
            self.assertIs(call["truncation"], False)

    def test_token_fit_fails_instead_of_truncating_essential_reference(self):
        with self.assertRaises(ValueError):
            fit_context(sample_context(), MarkerTokenizer(), max_input_tokens=100)

    def test_citation_audit_reports_unknown_and_missing_references(self):
        sources = sample_context()["sources"]
        result = citation_audit("查询帧 [S2]；额外断言 [S99]。", sources)
        self.assertEqual(set(result["known_ids"]), {"S1", "S2"})
        self.assertEqual(set(result["cited_ids"]), {"S2", "S99"})
        self.assertEqual(result["unknown_ids"], ["S99"])
        self.assertIs(result["has_citations"], True)
        self.assertIs(result["all_citations_known"], False)
        missing = citation_audit("没有提供依据的答案", sources)
        self.assertIs(missing["has_citations"], False)


if __name__ == "__main__":
    unittest.main()
