"""Optional synchronized CUDA boundaries; importing this module never uses a GPU."""

from contextlib import contextmanager

if __package__:
    from .run_code_training_preflight import gpu_memory
else:
    from run_code_training_preflight import gpu_memory


@contextmanager
def training_operation(torch, event, stage, enabled=False, **context):
    """Attribute asynchronous failures without changing the training calculation."""
    if not enabled:
        yield
        return

    def observe(phase):
        torch.cuda.synchronize(0)
        memory = gpu_memory(torch)
        stats = torch.cuda.memory_stats(0)
        memory.update(
            active_mib=round(stats.get("active_bytes.all.current", 0) / 2**20, 2),
            inactive_split_mib=round(stats.get("inactive_split_bytes.all.current", 0) / 2**20, 2),
        )
        event(stage, phase=phase, memory=memory, **context)

    event(stage, phase="start", **context)
    observe("ready")
    yield
    observe("complete")
