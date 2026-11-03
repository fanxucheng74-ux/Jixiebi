"""Finite, deterministic training groups and supervised-token denominators.

This module has no framework, filesystem, network or device access. It returns
sample IDs only and never changes the examples or supplies training answers.
"""

from collections.abc import Mapping
import random


def _bounded_int(value, name, minimum, maximum):
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError(f"{name} must be an integer in {minimum}..{maximum}")


def _example_fields(example):
    if not isinstance(example, dict):
        raise ValueError("each training example must be an object")
    sample_id = example.get("id")
    if not isinstance(sample_id, str) or not sample_id.strip():
        raise ValueError("training example ID must be nonempty text")
    if example.get("split") != "train":
        raise ValueError("only train examples may enter the optimizer schedule")
    sequence = example.get("sequence_tokens")
    completion = example.get("completion_tokens")
    _bounded_int(sequence, "sequence_tokens", 1, 4096)
    _bounded_int(completion, "completion_tokens", 1, sequence)
    return sample_id, sequence, completion


def make_epoch_groups(examples, epochs=3, accumulation_steps=4, seed=42):
    """Schedule every sample once per epoch, with the longest first on epoch1.

    Each epoch uses its own Random(seed + epoch - 1). The first epoch moves
    only its deterministically selected longest ID to the front, preserving
    all other shuffled relative positions. A short final group is retained.
    """
    _bounded_int(epochs, "epochs", 1, 5)
    _bounded_int(accumulation_steps, "accumulation_steps", 1, 16)
    if type(seed) is not int:
        raise ValueError("seed must be an integer, not bool")
    if not isinstance(examples, (list, tuple)) or not examples:
        raise ValueError("examples must be a nonempty finite list or tuple")
    checked = [_example_fields(example) for example in examples]
    ids = [row[0] for row in checked]
    if len(set(ids)) != len(ids):
        raise ValueError("training sample IDs must be unique")
    longest = max(checked, key=lambda row: (row[1], row[0]))[0]
    schedule = []
    for epoch in range(1, epochs + 1):
        shuffled = list(ids)
        random.Random(seed + epoch - 1).shuffle(shuffled)
        if epoch == 1:
            shuffled.remove(longest)
            shuffled.insert(0, longest)
        groups = [
            shuffled[start : start + accumulation_steps]
            for start in range(0, len(shuffled), accumulation_steps)
        ]
        schedule.append({"epoch": epoch, "groups": groups})
    return schedule


def group_supervised_tokens(group_ids, examples_by_id):
    """Return the denominator for sum(loss_i * supervised_i) / group total.

    Call backward for each microbatch after weighting its mean causal loss by
    completion_tokens / this denominator. This includes a short final group.
    """
    if not isinstance(group_ids, (list, tuple)) or not group_ids:
        raise ValueError("group IDs must be a nonempty finite list or tuple")
    if not isinstance(examples_by_id, Mapping):
        raise ValueError("examples_by_id must be a mapping")
    if any(not isinstance(sample_id, str) or not sample_id.strip() for sample_id in group_ids):
        raise ValueError("group sample IDs must be nonempty text")
    if len(set(group_ids)) != len(group_ids):
        raise ValueError("group contains duplicate sample IDs")
    total = 0
    for sample_id in group_ids:
        if sample_id not in examples_by_id:
            raise ValueError("unknown training sample ID: " + sample_id)
        actual_id, _, completion = _example_fields(examples_by_id[sample_id])
        if actual_id != sample_id:
            raise ValueError("mapping key differs from training sample ID")
        total += completion
    return total
