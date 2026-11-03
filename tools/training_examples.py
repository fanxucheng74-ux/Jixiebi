"""Pure Python supervised-label preparation and token-weighted validation metrics.

No model, framework, network, filesystem or device access is performed here.
Labels use the model's standard causal loss shift; this module never shifts them.
"""

import math


def encode_example(record, tokenizer, max_length=4096):
    """Keep a complete chat example and mask only its exact prompt token prefix."""
    if type(max_length) is not int or max_length < 1:
        raise ValueError("max_length must be a positive integer")
    if not isinstance(record, dict):
        raise ValueError("record must be an object")
    for field in ("id", "split", "category"):
        if not isinstance(record.get(field), str) or not record[field].strip():
            raise ValueError("missing or empty record field: " + field)
    if record["split"] not in ("train", "validation", "test"):
        raise ValueError("unknown example split")
    messages = record.get("messages")
    if not isinstance(messages, list) or len(messages) != 3:
        raise ValueError("expected exactly one system/user/assistant exchange")
    if any(not isinstance(message, dict) for message in messages):
        raise ValueError("each message must be an object")
    if [message.get("role") for message in messages] != ["system", "user", "assistant"]:
        raise ValueError("expected system/user/assistant role order")
    if any(
        not isinstance(message.get("content"), str) or not message["content"].strip()
        for message in messages
    ):
        raise ValueError("messages must have nonempty text content")

    full = tokenizer.apply_chat_template(
        messages, tokenize=True, add_generation_prompt=False, truncation=False
    )
    prefix = tokenizer.apply_chat_template(
        messages[:-1], tokenize=True, add_generation_prompt=True, truncation=False
    )
    if not isinstance(full, list) or not isinstance(prefix, list):
        raise ValueError("chat template must return plain token ID lists")
    if any(type(token) is not int or token < 0 for token in full + prefix):
        raise ValueError("token IDs must be nonnegative integers, not bools")
    if not prefix or full[: len(prefix)] != prefix:
        raise ValueError("tokenized prompt is not an exact full-sequence prefix")
    if len(full) == len(prefix):
        raise ValueError("assistant completion contains no tokens")
    if len(full) > max_length:
        raise ValueError("complete example exceeds max_length; truncation is disabled")

    input_ids = list(full)
    labels = list(full)
    labels[: len(prefix)] = [-100] * len(prefix)
    return {
        "id": record["id"],
        "split": record["split"],
        "category": record["category"],
        "input_ids": input_ids,
        "attention_mask": [1] * len(full),
        "labels": labels,
        "prompt_tokens": len(prefix),
        "completion_tokens": len(full) - len(prefix),
        "sequence_tokens": len(full),
    }


def summarize_validation(rows):
    """Aggregate each example's causal loss by its supervised token count."""
    rows = list(rows)
    if not rows:
        raise ValueError("validation rows cannot be empty")
    seen, checked = set(), []
    for row in rows:
        if not isinstance(row, dict):
            raise ValueError("validation row must be an object")
        sample_id = row.get("id")
        if not isinstance(sample_id, str) or not sample_id.strip() or sample_id in seen:
            raise ValueError("validation sample IDs must be nonempty and unique")
        seen.add(sample_id)
        count, loss = row.get("supervised_tokens"), row.get("loss")
        if type(count) is not int or count < 1:
            raise ValueError("supervised_tokens must be a positive integer")
        if type(loss) not in (int, float):
            raise ValueError("loss must be a finite nonnegative number")
        try:
            loss = float(loss)
        except OverflowError as error:
            raise ValueError("loss must be a finite nonnegative number") from error
        if not math.isfinite(loss) or loss < 0:
            raise ValueError("loss must be a finite nonnegative number")
        checked.append((count, loss))
    total_tokens = sum(count for count, _ in checked)
    # Normalize before multiplication so large token counts do not inflate loss.
    weighted_loss = math.fsum(loss * (count / total_tokens) for count, loss in checked)
    if not math.isfinite(weighted_loss):
        raise ValueError("token-weighted loss is not finite")
    try:
        perplexity = math.exp(weighted_loss)
    except OverflowError:
        perplexity = None
    return {
        "count": len(rows),
        "total_supervised_tokens": total_tokens,
        "token_weighted_loss": weighted_loss,
        "perplexity": perplexity,
        "perplexity_overflow": perplexity is None,
    }
