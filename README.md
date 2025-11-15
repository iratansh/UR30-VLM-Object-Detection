# UR30 Vision-Language Manipulation Research Platform

A research-focused ROS2 + Gazebo stack for studying how a UR30 manipulator can follow
natural-language commands by combining speech understanding, vision-language models,
depth reasoning, and hybrid inverse kinematics. This repository currently targets a
single validated scenario: **picking up the green book in simulation via
`vision/scripts/test_pick_green_book.py`**. Everything else (full HRI workflow,
construction-domain clarifications, advanced grasp policies, etc.) remains a work in
progress and is being handed off for continued research.

> **Status snapshot (April 2024 handoff)**
>
> - ✅ Baseline scenario: `test_pick_green_book.py` succeeds against Gazebo + UR30 +
>   simulated RealSense using color-based detection.
> - ⚠️ Remainder of the Unified Vision System is feature-complete but not fully
>   validated; many components depend on unavailable hardware or pending calibration.
> - 🎓 Codebase is part of an active study. Expect experimental scripts, TODOs, and
>   partially stubbed integrations.

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Repository Layout](#repository-layout)
3. [Environment &amp; Setup](#environment--setup)
4. [Simulation &amp; Baseline Workflow](#simulation--baseline-workflow)
5. [Testing](#testing)
6. [Key Python Modules](#key-python-modules)
7. [Research Backlog &amp; Non-complete Features](#research-backlog--non-complete-features)
8. [Handoff Checklist](#handoff-checklist)

---

## System Overview

The stack is organized around the `vision/unified_vision_system` package, which hosts
all perception, calibration, HRI, and control code. Supporting scripts (launch files,
diagnostics, Gazebo helpers) live under `vision/scripts` and `vision/launch`. Docker
wrappers plus `ur30_vlm_environment.yml` define the Conda environment used for local
development.

High-level data flow for the validated baseline:

```
Speech cmd (optional) ─┐
                      │            ┌─────────────────────┐
Gazebo UR30 + RealSense ──► ROS2 ──►│ UnifiedVisionSystem │──► Hybrid IK / UR30 action
(Color + depth feeds)  │           └─────────────────────┘
                      │                   │
              test_pick_green_book.py     ▼
                (color detector)    UR30 motion + gripper
```

## Repository Layout

| Path                                                          | Purpose                                                                               |
| ------------------------------------------------------------- | ------------------------------------------------------------------------------------- |
| `vision/unified_vision_system/`                             | Core package (perception, calibration, HRI, control, metrics, system wrappers).       |
| `vision/scripts/`                                           | Operational scripts (baseline tests, diagnostics, launch helpers, calibration tools). |
| `vision/launch/`                                            | Launch descriptions for Gazebo + ROS2 orchestration.                                  |
| `tests/unit`                                                | Fast unit tests for managers/controllers.                                             |
| `tests/integration`                                         | Multi-component integration tests.                                                    |
| `tests/system/vision`                                       | Former `vision/testing` suite: Gazebo, hardware, and ROS-dependent scenarios.       |
| `tools/`                                                    | Utility scripts (e.g.,`relativize_colcon_symlinks.py`).                             |
| `start-docker.sh`, `launch_*`, `restart_gazebo_ur30.sh` | Convenience scripts for starting the dev container and Gazebo sessions.               |
| `TODO_UPDATED.md`                                           | Rolling research backlog with validation notes and next steps.                        |

Refer to `vision/README.md` for a deeper dive into the `vision/` package structure.

## Environment & Setup

1. **Clone & submodules** (if any):

   ```bash
   git clone git@github.com:YOUR_ORG/UR5e-VLM-Object-Detection.git
   cd UR5e-VLM-Object-Detection
   ```
2. **Create the Conda environment** (Python 3.10):

   ```bash
   conda env create -f ur30_vlm_environment.yml
   conda activate ur30_vlm_environment
   ```
3. **Docker-based workflow (recommended)**:

   ```bash
   ./start-docker.sh                   # Launch dev container (UR5e-VLM working image)
   ./setup_environment.sh              # Install ROS2 overlay + pip deps inside the container
   ./launch_sim.sh                     # Quick helper that launches Gazebo + supporting nodes
   ```
4. **Native ROS2 setup** (if running outside Docker): source ROS2 Humble, ensure the
   workspace is built (`colcon build`), and export `PYTHONPATH` to include the `vision/`
   directory before running any scripts.

## Simulation & Baseline Workflow

The current, validated workflow targets the **green book pick-up baseline**:

1. **Start Gazebo**

   ```bash
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ros2 launch unified_vision_system launch_gazebo_with_red_cube.py use_rviz:=false use_moveit:=false
   ```
2. **Run the baseline script** (terminal #2)

   ```bash
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   python3.10 vision/scripts/test_pick_green_book.py
   ```
3. **What it does**

   - Subscribes to the simulated RealSense RGB + depth feeds.
   - Performs color-based segmentation tuned for Gazebo’s green book.
   - Converts depth to 3D, solves IK with `UR30Kinematics`, and commands the UR controller.
   - Prints detailed progress logs and halts if safety thresholds are violated.
4. **Troubleshooting**

   - Use `vision/scripts/save_camera_view.py` to confirm the book is visible.
   - Run `vision/scripts/check_camera_tf.py` to validate TF tree integrity.
   - Restart Gazebo with `restart_gazebo_ur30.sh` if controllers freeze.

## Testing

All automated tests have moved under `tests/`:

| Suite           | Location                | Notes                                                                                                                                                                               |
| --------------- | ----------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Unit            | `tests/unit`          | Pure-Python checks for TTS, RAG, controllers, etc. Run with `python -m pytest tests/unit`.                                                                                        |
| Integration     | `tests/integration`   | Scenario tests that exercise multi-component logic without ROS.                                                                                                                     |
| System / Vision | `tests/system/vision` | Former `vision/testing` folder. Requires ROS2 + (often) Gazebo or RealSense. Run inside Docker with `conda run -n ur5e_vlm_environment python tests/system/vision/<script>.py`. |

Common commands (inside the container):

```bash
conda activate ur5e_vlm_environment
python -m pytest tests/unit
python tests/system/vision/test_workspace_validator_standalone.py
```

See `TODO_UPDATED.md` for the canonical list of ROS-dependent tests and their status.

## Key Python Modules

| Path                                                                     | Role                                                                      |
| ------------------------------------------------------------------------ | ------------------------------------------------------------------------- |
| `vision/unified_vision_system/system/UnifiedVisionSystem.py`           | Primary ROS2 node integrating speech, perception, depth, IK, and control. |
| `vision/unified_vision_system/system/UnifiedVisionSystemSim.py`        | Simulation-aware subclass with Gazebo camera bridges.                     |
| `vision/unified_vision_system/control/UR30Kinematics.py`               | Robotics Toolbox-backed IK/FK with fallbacks.                             |
| `vision/unified_vision_system/control/HybridIKWrapper.py`              | VLM-friendly IK facade with approximation logic.                          |
| `vision/unified_vision_system/perception/OWLViTDetector.py`            | Visual-language model detector wrapper.                                   |
| `vision/unified_vision_system/perception/DepthAwareDetector.py`        | Depth fusion + 3D localization.                                           |
| `vision/unified_vision_system/perception/GraspPointDetector.py`        | Determines viable grasp poses from 2D/3D cues.                            |
| `vision/unified_vision_system/perception/WorkSpaceValidator.py`        | Safety boundaries + reachability enforcement.                             |
| `vision/unified_vision_system/hri/SpeechCommandProcessor.py`           | Whisper + spaCy pipeline for natural-language intents.                    |
| `vision/unified_vision_system/hri/ConstructionClarificationManager.py` | Clarification strategies for the study.                                   |
| `vision/unified_vision_system/hri/EnhancedConstructionRAG.py`          | ChromaDB/SentenceTransformer-based retrieval system.                      |
| `vision/unified_vision_system/hri/ConstructionTTSManager.py`           | Voice feedback tuned for construction environments.                       |
| `vision/unified_vision_system/hri/ExperimentalController.py`           | Experimental protocol orchestrator (A/B testing, questionnaires).         |
| `vision/scripts/test_pick_green_book.py`                               | Baseline scenario: picks up the green book in Gazebo.                     |
| `tests/system/vision/test_ur30_core_functionality.py`                  | Smoke test for kinematics, workspace validator, OWL-ViT, etc.             |

Each of these files now begins with descriptive module docstrings plus inline comments where logic is non-obvious; rely on them when onboarding new collaborators.

## Research Backlog & Non-complete Features

This repo intentionally contains unfinished or blocked items—see `TODO_UPDATED.md` for
daily notes. Highlights:

- Camera calibration standalone tests (`tests/system/vision/test_camera_calibration_standalone.py`) are blocked by ROS2 dependencies; must be validated in a full ROS session.
- Grasp + Hybrid IK refinements are pending hardware time.
- Advanced HRI experiments (clarification A/B tests, trust metrics) depend on real
  participants and have mock fallbacks for offline development.
- Documentation for additional ROS launch topologies (`vision/scripts/test_gazebo_simulation.launch.py`) still needs verification once MoveIt integration stabilizes.

Treat the backlog as the authoritative roadmap—update it when you validate or change
any subsystem.

## Handoff Checklist

Before continuing development, the new researcher should:

1. **Re-create the environment** using `ur30_vlm_environment.yml` and confirm
   `test_pick_green_book.py` succeeds inside Docker.
2. **Review key modules** (table above) to understand perception, IK, and HRI flows.
3. **Read `TODO_UPDATED.md`** to internalize pending experiments and known gaps.
4. **Validate safety**: run `tests/system/vision/test_workspace_validator_standalone.py` and `test_ur30_kinematics_standalone.py` once SciPy/LAPACK dependencies are satisfied.
5. **Document findings**: keep `tests/README.md` and the TODO file current as you expand coverage.
6. **Plan next experiments** in coordination with the larger research team; this repo
   intentionally stays flexible to accommodate protocol changes.

Please keep augmenting the documentation as the study evolves!
