"""Supervised target masks and metric weighting; no ML packages or hardware."""

from copy import deepcopy
import math
import unittest

from tools.training_examples import encode_example, summarize_validation


def record():
    return {
        "id": "example",
        "split": "train",
        "category": "fixture",
        "messages": [
            {"role": "system", "content": "约定"},
            {"role": "user", "content": "需求"},
            {"role": "assistant", "content": "实现"},
        ],
    }


class FakeTokenizer:
    def __init__(self, full=None, prefix=None):
        self.full = [1, 11, 12, 13, 21, 22, 99, 10] if full is None else full
        self.prefix = [1, 11, 12, 13] if prefix is None else prefix
        self.calls = []

    def apply_chat_template(self, messages, *, tokenize, add_generation_prompt, truncation):
        self.calls.append((len(messages), tokenize, add_generation_prompt, truncation))
        return self.prefix if add_generation_prompt else self.full


class TrainingExampleTests(unittest.TestCase):
    def test_masks_prompt_only_preserving_answer_eos_and_template_tail(self):
        example, tokenizer = record(), FakeTokenizer()
        original = deepcopy(example)
        encoded = encode_example(example, tokenizer)
        self.assertEqual(encoded["input_ids"], [1, 11, 12, 13, 21, 22, 99, 10])
        self.assertEqual(encoded["labels"], [-100, -100, -100, -100, 21, 22, 99, 10])
        self.assertEqual(encoded["attention_mask"], [1] * 8)
        self.assertEqual(encoded["prompt_tokens"], 4)
        self.assertEqual(encoded["completion_tokens"], 4)
        self.assertEqual(encoded["sequence_tokens"], 8)
        self.assertEqual(example, original)
        self.assertEqual(tokenizer.full, [1, 11, 12, 13, 21, 22, 99, 10])
        self.assertEqual(tokenizer.calls, [(3, True, False, False), (2, True, True, False)])

    def test_exact_length_allowed_but_overlong_example_not_truncated(self):
        tokenizer = FakeTokenizer()
        self.assertEqual(encode_example(record(), tokenizer, max_length=8)["sequence_tokens"], 8)
        with self.assertRaisesRegex(ValueError, "truncation is disabled"):
            encode_example(record(), tokenizer, max_length=7)
        self.assertEqual(len(tokenizer.full), 8)
        for invalid in (0, -1, True, 8.0):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                encode_example(record(), tokenizer, max_length=invalid)

    def test_rejects_prefix_mismatch_empty_prefix_and_empty_completion(self):
        for tokenizer in (
            FakeTokenizer(prefix=[1, 88]),
            FakeTokenizer(prefix=[]),
            FakeTokenizer(full=[1, 11, 12, 13]),
        ):
            with self.subTest(tokenizer=tokenizer), self.assertRaises(ValueError):
                encode_example(record(), tokenizer)

    def test_rejects_bool_negative_float_and_tensor_like_token_ids(self):
        for invalid in (True, -1, 2.5):
            tokenizer = FakeTokenizer(full=[1, 11, 12, 13, invalid])
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                encode_example(record(), tokenizer)
        with self.assertRaises(ValueError):
            encode_example(record(), FakeTokenizer(full=(1, 11, 12, 13, 21)))

    def test_rejects_invalid_messages_and_record_metadata(self):
        for field, value in (("id", ""), ("split", "unknown"), ("category", " ")):
            changed = record()
            changed[field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                encode_example(changed, FakeTokenizer())
        changed = record()
        changed["messages"][1]["role"] = "assistant"
        with self.assertRaises(ValueError):
            encode_example(changed, FakeTokenizer())
        changed = record()
        changed["messages"][-1]["content"] = " "
        with self.assertRaises(ValueError):
            encode_example(changed, FakeTokenizer())

    def test_validation_loss_is_token_weighted_not_example_averaged(self):
        summary = summarize_validation(
            [
                {"id": "short", "loss": 1.0, "supervised_tokens": 1},
                {"id": "long", "loss": 3.0, "supervised_tokens": 3},
            ]
        )
        self.assertEqual(summary["count"], 2)
        self.assertEqual(summary["total_supervised_tokens"], 4)
        self.assertEqual(summary["token_weighted_loss"], 2.5)
        self.assertAlmostEqual(summary["perplexity"], math.exp(2.5))
        self.assertFalse(summary["perplexity_overflow"])

    def test_perplexity_overflow_is_reported_without_infinite_json_values(self):
        summary = summarize_validation([{"id": "huge", "loss": 1000.0, "supervised_tokens": 2}])
        self.assertEqual(summary["token_weighted_loss"], 1000.0)
        self.assertIsNone(summary["perplexity"])
        self.assertTrue(summary["perplexity_overflow"])

    def test_rejects_empty_duplicate_nan_and_invalid_validation_rows(self):
        valid = {"id": "a", "loss": 1.0, "supervised_tokens": 2}
        with self.assertRaises(ValueError):
            summarize_validation([])
        with self.assertRaises(ValueError):
            summarize_validation([valid, valid.copy()])
        for invalid in (float("nan"), float("inf"), -0.1, True, "1.0"):
            with self.subTest(loss=invalid), self.assertRaises(ValueError):
                summarize_validation([dict(valid, loss=invalid)])
        for invalid in (0, -1, True, 1.5):
            with self.subTest(supervised_tokens=invalid), self.assertRaises(ValueError):
                summarize_validation([dict(valid, supervised_tokens=invalid)])


if __name__ == "__main__":
    unittest.main()
