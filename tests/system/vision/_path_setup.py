"""
Shared path setup for the relocated vision system tests.
Ensures the repository root and the vision package are importable.
"""

from pathlib import Path
import sys


def setup_test_paths() -> None:
    """Add the repository root and vision package directories to sys.path."""
    current_dir = Path(__file__).resolve().parent
    repo_root = current_dir.parents[2]
    vision_dir = repo_root / "vision"
    src_dir = repo_root / "src"

    for path in (repo_root, vision_dir, src_dir):
        if not path.exists():
            continue
        path_str = str(path)
        if path_str not in sys.path:
            sys.path.insert(0, path_str)

