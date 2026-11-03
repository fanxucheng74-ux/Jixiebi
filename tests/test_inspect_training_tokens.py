"""Token counting tests use a deterministic fake; no downloads or ML backends."""

import contextlib
from copy import deepcopy
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from tools.inspect_training_tokens import (
    count_records,
    load_records,
    load_tokenizer,
    main,
    summarize,
)


class FakeTokenizer:
    eos_token_id = 99

    def apply_chat_template(self, messages, *, tokenize, add_generation_prompt, truncation):
        assert tokenize and not truncation
        values = [1]
        for message in messages:
            values += [2, *[ord(char) for char in message["content"]], 99]
        if add_generation_prompt:
            values += [2]
        return values


def rows():
    return [
        {
            "id": split,
            "split": split,
            "category": "fixture",
            "source": {"path": "fixture.py"},
            "messages": [
                {"role": "system", "content": "约定"},
                {"role": "user", "content": "任务"},
                {"role": "assistant", "content": "完整代码"},
            ],
        }
        for split in ("train", "validation", "test")
    ]


def fixture(folder):
    dataset = folder / "dataset"
    dataset.mkdir()
    records = rows()
    (dataset / "records.jsonl").write_text(
        "\n".join(json.dumps(row) for row in records), encoding="utf-8"
    )
    for record in records:
        (dataset / (record["split"] + ".messages.jsonl")).write_text(
            json.dumps({"messages": record["messages"]}), encoding="utf-8"
        )
    return dataset


class TokenInspectionTests(unittest.TestCase):
    def test_missing_jinja_fails_before_hub_import_or_download(self):
        with patch("tools.inspect_training_tokens.find_spec", return_value=None):
            with self.assertRaisesRegex(ImportError, "python -m pip install"):
                load_tokenizer("unused/model", "main")

    def test_control_tokens_and_prefix_are_counted_without_truncation(self):
        counted = count_records(rows(), FakeTokenizer())
        self.assertEqual(counted[0]["prompt_tokens"], 10)
        self.assertEqual(counted[0]["completion_tokens_including_template_end"], 5)
        self.assertEqual(counted[0]["total_tokens"], 15)
        self.assertTrue(counted[0]["completion_contains_eos"])
        self.assertEqual(summarize(counted)["train"]["max_total_tokens"], 15)

    def test_fails_if_tokenized_prompt_is_not_the_full_sequence_prefix(self):
        class BadTokenizer(FakeTokenizer):
            def apply_chat_template(self, messages, **kwargs):
                result = super().apply_chat_template(messages, **kwargs)
                return result + [771] if kwargs["add_generation_prompt"] else result

        with self.assertRaisesRegex(ValueError, "prefix differs"):
            count_records(rows(), BadTokenizer())

    def test_rejects_divergent_export_and_empty_or_invalid_messages(self):
        with tempfile.TemporaryDirectory() as folder:
            dataset = fixture(Path(folder))
            self.assertEqual(len(load_records(dataset)[0]), 3)
            path = dataset / "train.messages.jsonl"
            path.write_text('{"messages": []}', encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "differs"):
                load_records(dataset)

    def test_lengths_at_threshold_and_over_threshold(self):
        sample = rows()
        sample[0]["messages"][-1]["content"] = "x" * (2048 - 11)
        data = count_records(sample, FakeTokenizer())
        self.assertEqual(data[0]["total_tokens"], 2048)
        self.assertEqual(summarize(data)["train"]["exceeds_sequence_length"]["2048"], 0)
        sample[0]["messages"][-1]["content"] += "x"
        self.assertEqual(
            summarize(count_records(sample, FakeTokenizer()))["train"]["exceeds_sequence_length"][
                "2048"
            ],
            1,
        )

    def test_cli_writes_report_never_overwrites_or_changes_dataset(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            dataset = fixture(root)
            before = {p.name: p.read_bytes() for p in dataset.iterdir()}
            output = root / "report"
            args = ["--dataset", str(dataset), "--output", str(output)]
            with (
                patch(
                    "tools.inspect_training_tokens.load_tokenizer",
                    return_value=(FakeTokenizer(), {}),
                ) as loader,
                contextlib.redirect_stdout(io.StringIO()),
            ):
                self.assertEqual(main(args), 0)
                loader.assert_called_once()
                with self.assertRaises(FileExistsError):
                    main(args)
                loader.assert_called_once()
            report = json.loads((output / "report.json").read_text(encoding="utf-8"))
            self.assertFalse(report["truncation"])
            self.assertFalse(report["training_launched"])
            self.assertEqual({p.name: p.read_bytes() for p in dataset.iterdir()}, before)

    def test_cli_refuses_output_inside_frozen_dataset_before_loading_tokenizer(self):
        with tempfile.TemporaryDirectory() as folder:
            dataset = fixture(Path(folder))
            with patch("tools.inspect_training_tokens.load_tokenizer") as loader:
                with self.assertRaises(ValueError):
                    main(["--dataset", str(dataset), "--output", str(dataset / "report")])
                loader.assert_not_called()

    def test_empty_completion_is_rejected(self):
        class EmptyTokenizer(FakeTokenizer):
            def apply_chat_template(self, messages, **kwargs):
                if messages[-1]["role"] == "assistant":
                    messages = messages[:-1]
                    kwargs["add_generation_prompt"] = True
                return super().apply_chat_template(messages, **kwargs)

        with self.assertRaisesRegex(ValueError, "empty assistant"):
            count_records(deepcopy(rows()), EmptyTokenizer())


if __name__ == "__main__":
    unittest.main()
