# Tests Overview

All tests now live under the top-level `tests/` directory:

- `tests/unit`: fast module-level checks for speech, RAG, and controller utilities.
- `tests/integration`: multi-component scenarios that exercise construction workflows.
- `tests/system/vision`: the former `vision/testing` suite covering hardware, Gazebo, and end-to-end validation of the vision stack.

Each system-level test automatically ensures the repository root and `vision/` package are on `PYTHONPATH` via `_path_setup.py`, so they can still be executed directly:

```bash
conda run -n ur5e_vlm_environment python tests/system/vision/test_workspace_validator_standalone.py
```

Refer to `TODO_UPDATED.md` for detailed instructions on the highest-priority system tests to run.
