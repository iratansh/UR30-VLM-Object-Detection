# UR30 Vision-Language Research TODO

**Last Updated:** January 2026 handoff

This repo reflects the in-progress UR30 manipulation study. The foundations are stable,
but real-world validation still depends on the Gazebo + color-detection baseline
(`vision/scripts/test_pick_green_book.py`). Use this document as the lightweight plan
for the incoming researcher/s.

---

## Verified Components

| Area                     | Notes                                                                                                                                                    |
| ------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Hybrid IK Wrapper        | `vision/unified_vision_system/control/HybridIKWrapper.py` tested against UR30Kinematics; fallback logic and adaptive timeouts confirmed in simulation. |
| Unit Tests               | `tests/unit` suite runs clean inside `ur30_vlm_environment`; covers TTS, RAG, controllers, and shared managers.                                      |
| RAG & TTS Pipeline       | `EnhancedConstructionRAG`, `ConstructionTTSManager`, and the experimental controller path were exercised via the macOS and research harness scripts. |
| OWL-ViT Detector         | `tests/system/vision/test_owlvit_detector_standalone.py` passes (100%); model loads successfully in the Conda env.                                     |
| Speech Command Processor | `unified_vision_system/hri/SpeechCommandProcessor.py` validated end-to-end from the test harness; Whisper + spaCy fallback workflow confirmed.         |

Everything above is ready for integration once we are confident in the Gazebo workflow.

---

## Primary Handoff Task - Gazebo Baseline Verification

Goal: **Confirm the system can pick up the green book in simulation using the
color-based perception pipeline.** This is the gating step before re-enabling the full
OWL-ViT + speech pipeline on hardware.

### Why color-based detection?

- OWL-ViT models are tuned for real-world textures and perform poorly on Gazebo's
  low-fidelity renders.
- `vision/scripts/test_pick_green_book.py` implements a deterministic color detector that
  matches the book in the default UR30 camera FOV, letting us validate motion planning,
  IK, grasp execution, and controller wiring without fighting domain gaps.

### Expectations for the next researcher

1. **Run Gazebo + UR30 + RealSense simulation** via `ros2 launch unified_vision_system launch_gazebo_with_red_cube.py use_rviz:=false use_moveit:=false`.
2. **Execute** `test_pick_green_book.py` inside the same workspace
   (ROS2 Humble sourced, `ur30_vlm_environment` active) and confirm the robot successfully
   grasps and lifts the book.
3. **If needed**, create minimal variants of this test (e.g., different colors/poses) to
   exercise additional aspects of the perception -> IK -> motion chain before deploying the
   more complex OWL-ViT + speech stack on real hardware.
4. **Document findings** in this TODO file and/or `tests/README.md` so we maintain a clear
   record of which combinations were validated.

### Success criteria

- Camera topics publish reliably; `save_camera_view.py` shows the book in frame.
- Color segmentation finds the book; 3D pose is computed from depth data.
- UR30 executes the grasp through the scaled_joint_trajectory controller and lifts the
  object without violation of workspace constraints.
- Logs confirm the perception, IK, and execution stages all pass without triggering
  fallback errors.

---

## After the baseline passes...

Once the green-book scenario works reliably, proceed with:

1. **Re-enabling the full OWL-ViT + speech pipeline** for hardware trials.
2. **Running the ROS-dependent system tests** (`tests/system/vision/*.py`) inside Gazebo or
   on-device as appropriate.
3. **Updating the research protocol scripts** (clarification manager, experimental
   controller) with any newly validated behaviors.

Keep this file updated as you progress.
