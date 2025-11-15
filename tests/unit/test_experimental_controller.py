import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
VISION = ROOT / "vision"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(VISION) not in sys.path:
    sys.path.insert(0, str(VISION))

from ExperimentalController import (  # noqa: E402
    ConstructionExperimentalController,
    ExperimentCondition,
    ExperimentPhase,
    ExperimentalDesign,
    ParticipantProfile,
)
from ConstructionClarificationManager import UserExpertiseLevel  # noqa: E402


def test_experimental_controller_workflow(tmp_path):
    data_dir = tmp_path / "data"
    controller = ConstructionExperimentalController(
        study_name="Unit Test Study",
        data_directory=str(data_dir),
        enable_real_time_analysis=False,
    )

    design = ExperimentalDesign(
        study_name="Unit Test Study",
        conditions=[
            ExperimentCondition.CONTROL_DIRECT,
            ExperimentCondition.TREATMENT_CONFIDENCE,
        ],
        block_randomization=False,
        within_subject=True,
        tasks_per_condition=2,
        randomization_seed=123,
    )
    controller.configure_experiment(design)

    participant = ParticipantProfile(
        participant_id="P001",
        expertise_level=UserExpertiseLevel.JOURNEYMAN,
        construction_experience_years=6,
    )
    pid = controller.enroll_participant(participant)
    assert pid == "P001"

    session_id = controller.start_experimental_session(pid, condition_index=0)
    assert session_id.startswith("P001_S1_")

    assert controller.advance_to_phase(ExperimentPhase.PRE_ASSESSMENT)
    assert controller.advance_to_phase(ExperimentPhase.MAIN_TASK)

    assert controller.record_task_attempt(
        task_description="Identify framing hammer",
        success=True,
        interaction_time=12.5,
        clarifications_used=1,
    )
    controller.record_task_attempt(
        task_description="Identify square",
        success=False,
        interaction_time=14.0,
        clarifications_used=2,
    )

    assert controller.advance_to_phase(ExperimentPhase.POST_ASSESSMENT)
    assert controller.advance_to_phase(ExperimentPhase.DEBRIEFING)

    assert controller.end_experimental_session() is True

    summary = controller.get_experimental_summary()
    assert summary["total_sessions"] == 1
    assert summary["total_participants"] == 1

    dataset_path = controller.export_complete_dataset(str(tmp_path / "dataset.json"))
    with open(dataset_path) as handle:
        dataset = json.load(handle)
    assert dataset["sessions"]
    assert dataset["participants"]

    participant_files = list(data_dir.glob("participant_*.json"))
    session_files = list(data_dir.glob("session_*.json"))
    assert participant_files
    assert session_files

    controller.cleanup()
