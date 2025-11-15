# Vision Package Overview

This README supplements the root documentation by describing the layout and
responsibilities of the `vision/` workspace. Everything here ultimately feeds
the Unified Vision System ROS2 package that powers the UR30 manipulation study.

## Subpackages

| Path | Description |
| --- | --- |
| `unified_vision_system/control/` | UR30-specific IK, hybrid planners, and grasp controllers. |
| `unified_vision_system/perception/` | OWL-ViT wrapper, depth fusion, grasp-point reasoning, workspace validation. |
| `unified_vision_system/calibration/` | Eye-in-hand, hand-eye, and camera calibration utilities plus safety checkers. |
| `unified_vision_system/hri/` | Speech, clarification strategies, RAG knowledge retrieval, TTS, and experimental controllers. |
| `unified_vision_system/system/` | Primary ROS2 nodes (`UnifiedVisionSystem`, `UnifiedVisionSystemSim`). |
| `unified_vision_system/metrics/` | Trust, NASA TLX, and behavioral measurement helpers for the user study. |
| `scripts/` | Operational helpers (baseline tests, calibration scripts, Gazebo diagnostics). |
| `launch/` | Launch descriptions for Gazebo, MoveIt, and dedicated simulation setups. |

Every module includes a descriptive docstring plus inline comments for non-obvious
logic. When extending the stack, follow the same conventions to keep handoffs smooth.

## Key Workflows

1. **Baseline pick test** – `vision/scripts/test_pick_green_book.py`
   - Uses simulated RealSense RGB + depth, color-based detection, and `UR30Kinematics`
     to grab the green book in Gazebo.
   - Serves as the regression test for the current codebase state.

2. **Unified Vision System ROS2 node** – `vision/unified_vision_system/system/UnifiedVisionSystem.py`
   - Integrates speech, OWL-ViT, depth-aware detection, workspace validation, and
     hybrid IK.
   - `UnifiedVisionSystemSim` (same directory) overrides camera handling for Gazebo.

3. **Gazebo launch** – `vision/launch/launch_gazebo_with_red_cube.py`
   - Spins up UR30, RealSense, controllers, vision node, and optional MoveIt/RViz.

4. **Calibration & diagnostics**
   - `scripts/calibrate_eye_in_hand.py`, `scripts/check_camera_tf.py`,
     `scripts/save_camera_view.py` help verify hardware alignment.

## Extending the Package

- Keep ROS2 parameters documented at the top of each node.
- Update `TODO_UPDATED.md` whenever you add new tests, calibration files, or research
  hypotheses.
- If you introduce new dependencies, mirror them in `ur30_vlm_environment.yml` and the
  Docker workflow scripts.

This directory stays under active development—treat this README as a living guide and
expand it alongside your research.
