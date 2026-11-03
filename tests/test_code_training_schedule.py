"""Training membership, finite update counts and token-weighting without ML."""

from copy import deepcopy
import random
import unittest

from tools.code_training_schedule import group_supervised_tokens, make_epoch_groups


def examples(count=38):
    return [
        {
            "id": f"sample_{index:02d}",
            "split": "train",
            "sequence_tokens": index + 100,
            "completion_tokens": index + 1,
            "unrelated": [index],
        }
        for index in range(count)
    ]


def flattened(epoch):
    return [sample_id for group in epoch["groups"] for sample_id in group]


class TrainingScheduleTests(unittest.TestCase):
    def test_38_examples_once_per_epoch_last_group_two_and_total_30_updates(self):
        rows = examples()
        original = deepcopy(rows)
        schedule = make_epoch_groups(rows)
        self.assertEqual([epoch["epoch"] for epoch in schedule], [1, 2, 3])
        self.assertEqual(sum(len(epoch["groups"]) for epoch in schedule), 30)
        for epoch in schedule:
            self.assertEqual(len(epoch["groups"]), 10)
            self.assertEqual([len(group) for group in epoch["groups"]], [4] * 9 + [2])
            self.assertCountEqual(flattened(epoch), [row["id"] for row in rows])
            self.assertEqual(len(set(flattened(epoch))), 38)
        self.assertEqual(schedule[0]["groups"][0][0], "sample_37")
        self.assertEqual(rows, original)

    def test_longest_is_first_without_changing_other_shuffled_positions(self):
        rows = examples()
        ids = [row["id"] for row in rows]
        expected = list(ids)
        random.Random(42).shuffle(expected)
        expected.remove("sample_37")
        schedule = make_epoch_groups(rows)
        self.assertEqual(flattened(schedule[0]), ["sample_37", *expected])
        second = list(ids)
        random.Random(43).shuffle(second)
        self.assertEqual(flattened(schedule[1]), second)

    def test_deterministic_seed_tie_breaking_and_single_record(self):
        rows = examples(3)
        rows[0]["sequence_tokens"] = rows[2]["sequence_tokens"]
        schedule = make_epoch_groups(rows, seed=17)
        self.assertEqual(schedule, make_epoch_groups(deepcopy(rows), seed=17))
        self.assertEqual(schedule[0]["groups"][0][0], "sample_02")
        single = make_epoch_groups(examples(1), epochs=1, accumulation_steps=16)
        self.assertEqual(single, [{"epoch": 1, "groups": [["sample_00"]]}])

    def test_group_denominator_weights_tokens_and_handles_short_final_group(self):
        rows = examples(3)
        rows[0]["completion_tokens"] = 1
        rows[1]["completion_tokens"] = 9
        lookup = {row["id"]: row for row in rows}
        self.assertEqual(group_supervised_tokens(["sample_00", "sample_01"], lookup), 10)
        self.assertEqual(group_supervised_tokens(["sample_02"], lookup), 3)
        # Mean losses 10 and 0 must produce a token-weighted loss of1, not5.
        weighted_loss = (10 * 1 + 0 * 9) / group_supervised_tokens(
            ["sample_00", "sample_01"], lookup
        )
        self.assertEqual(weighted_loss, 1)

    def test_rejects_unbounded_or_invalid_schedule_parameters(self):
        for field, invalid in (
            ("epochs", 0),
            ("epochs", 6),
            ("epochs", True),
            ("accumulation_steps", 0),
            ("accumulation_steps", 17),
            ("accumulation_steps", False),
            ("seed", True),
            ("seed", 1.5),
        ):
            with self.subTest(field=field, invalid=invalid):
                with self.assertRaises(ValueError):
                    make_epoch_groups(examples(), **{field: invalid})
        for invalid in ([], (), None, "samples", iter(examples())):
            with self.assertRaises(ValueError):
                make_epoch_groups(invalid)

    def test_rejects_duplicate_ids_nontrain_and_invalid_lengths(self):
        duplicate = examples(2)
        duplicate[1]["id"] = duplicate[0]["id"]
        with self.assertRaises(ValueError):
            make_epoch_groups(duplicate)
        for field, invalid in (
            ("split", "validation"),
            ("split", "test"),
            ("id", ""),
            ("sequence_tokens", True),
            ("sequence_tokens", 0),
            ("sequence_tokens", 4097),
            ("completion_tokens", False),
            ("completion_tokens", 0),
            ("completion_tokens", 101),
        ):
            row = examples(1)
            row[0][field] = invalid
            with self.subTest(field=field, invalid=invalid):
                with self.assertRaises(ValueError):
                    make_epoch_groups(row)

    def test_group_rejects_unknown_duplicate_ids_and_bad_count_without_mutation(self):
        rows = examples(2)
        lookup = {row["id"]: row for row in rows}
        original = deepcopy(lookup)
        for ids in ([], ["missing"], ["sample_00", "sample_00"], [True], "sample_00"):
            with self.assertRaises(ValueError):
                group_supervised_tokens(ids, lookup)
        self.assertEqual(lookup, original)
        lookup["sample_00"]["completion_tokens"] = True
        with self.assertRaises(ValueError):
            group_supervised_tokens(["sample_00"], lookup)
        lookup["sample_00"]["completion_tokens"] = 1
        lookup["sample_00"]["id"] = "wrong_key"
        with self.assertRaises(ValueError):
            group_supervised_tokens(["sample_00"], lookup)


if __name__ == "__main__":
    unittest.main()
