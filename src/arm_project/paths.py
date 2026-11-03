"""Locate workspace data independently of the shell's current directory."""

import os
from pathlib import Path


def find_project_root() -> Path:
    """Require a marked workspace; never create run data in site-packages."""
    configured = os.environ.get("ARM_PROJECT_ROOT")
    if configured:
        root = Path(configured).expanduser().resolve(strict=True)
        if not (root / ".arm-project-root").is_file():
            raise ValueError("ARM_PROJECT_ROOT must point to a marked arm-project workspace")
        return root
    for root in (*Path(__file__).resolve().parents, Path.cwd(), *Path.cwd().parents):
        if (root / ".arm-project-root").is_file():
            return root
    raise RuntimeError("Workspace not found; set ARM_PROJECT_ROOT to the project directory")


PROJECT_ROOT = find_project_root()
