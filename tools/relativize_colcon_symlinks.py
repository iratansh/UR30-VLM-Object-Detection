"""Normalize colcon install symlinks so they remain valid after moving the workspace.

Colcon generates absolute symlinks inside the install tree when `--symlink-install`
(or the default for some packages) is used. If the workspace is relocated after
building—such as when the same tree is mounted inside a different Docker path—
those symlinks point at stale absolute paths (e.g. `/home/ur30_user/workspace`).

Running this script rewrites any symlink that points into the old workspace so
it becomes a relative link to the matching target inside the current repository.
That keeps `install/setup.bash` happy without requiring a rebuild.
"""

from __future__ import annotations

import os
from pathlib import Path


def find_repo_root(start: Path) -> Path:
    current = start.resolve()
    for parent in [current, *current.parents]:
        if (parent / ".git").is_dir():
            return parent
    raise RuntimeError("Could not locate repository root from" f" {start}")


def relativize_install_symlinks(install_root: Path) -> int:
    repo_root = find_repo_root(install_root)
    prefixes = (
        Path("/home/ur30_user/workspace"),
        Path("/workspace"),
    )
    fixed = 0

    for symlink in install_root.rglob("*"):
        if not symlink.is_symlink():
            continue

        target = Path(os.readlink(symlink))
        if not target.is_absolute():
            continue

        for prefix in prefixes:
            try:
                suffix = target.relative_to(prefix)
            except ValueError:
                continue

            candidate = (repo_root / suffix).resolve()
            if not candidate.exists():
                continue

            relative_target = os.path.relpath(candidate, symlink.parent)
            symlink.unlink()
            symlink.symlink_to(relative_target)
            fixed += 1
            break

    return fixed


def main() -> None:
    repo_root = find_repo_root(Path.cwd())
    install_root = repo_root / "install"
    if not install_root.is_dir():
        raise SystemExit(f"Install directory missing: {install_root}")

    fixed = relativize_install_symlinks(install_root)
    print(f"Fixed {fixed} stale symlinks under {install_root}")


if __name__ == "__main__":
    main()
