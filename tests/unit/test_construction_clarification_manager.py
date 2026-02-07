import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
VISION = ROOT / "vision"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(VISION) not in sys.path:
    sys.path.insert(0, str(VISION))

from ConstructionClarificationManager import (  # noqa: E402
    ClarificationStrategy,
    ConstructionClarificationManager,
    TaskMemory,
    UserExpertiseLevel,
)


def _make_manager(tmp_path):
    memory_file = tmp_path / "memory.json"
    manager = ConstructionClarificationManager(
        enable_rag=False,
        enable_tts=False,
        memory_file=str(memory_file),
    )
    return manager


def test_direct_strategy_handles_missing_confidences(tmp_path):
    manager = _make_manager(tmp_path)
    try:
        detections = [
            {"label": "claw hammer"},
            {"trade_term": "ball-peen hammer"},
        ]
        response = manager.request_clarification(
            tool_request="hammer",
            detected_objects=detections,
            confidence_scores=[],
            strategy=ClarificationStrategy.DIRECT,
        )

        assert response.strategy is ClarificationStrategy.DIRECT
        assert "multiple tools" in response.text.lower()
        assert response.confidence == pytest.approx(0.0)
        assert response.metadata["selected_tool"] in {"claw hammer", "ball-peen hammer"}
    finally:
        manager.cleanup()


def test_history_strategy_handles_empty_memory_entries(tmp_path):
    manager = _make_manager(tmp_path)
    try:
        manager.task_memory.append(TaskMemory(tool_name=None, action="pickup"))
        manager.task_memory.append(TaskMemory(tool_name="finish hammer", action="pickup"))

        detections = [
            {"trade_term": "finish hammer", "label": "finish hammer"}
        ]
        response = manager.request_clarification(
            tool_request="hammer",
            detected_objects=detections,
            confidence_scores=[0.72],
            strategy=ClarificationStrategy.HISTORY_AWARE,
        )

        assert response.strategy is ClarificationStrategy.HISTORY_AWARE
        assert "finish hammer" in response.text.lower()
        assert response.metadata.get("similarity_match") is True
    finally:
        manager.cleanup()


def test_apprentice_adaptation_includes_education(tmp_path):
    manager = _make_manager(tmp_path)
    try:
        manager.update_user_expertise(UserExpertiseLevel.APPRENTICE)
        detections = [
            {
                "trade_term": "framing hammer",
                "category": "striking_tool",
            }
        ]
        response = manager.request_clarification(
            tool_request="hammer",
            detected_objects=detections,
            confidence_scores=[0.9],
            strategy=ClarificationStrategy.EXPERTISE_ADAPTIVE,
        )

        metadata = response.metadata
        assert metadata.get("expertise_level") == "apprentice"
        assert metadata.get("educational_description")
        assert "framing hammer" in response.text.lower()
    finally:
        manager.cleanup()


def test_task_memory_persistence(tmp_path):
    manager = _make_manager(tmp_path)
    try:
        manager.update_task_memory("framing hammer", "pickup", success=True)
    finally:
        manager.cleanup()

    reloaded = ConstructionClarificationManager(
        enable_rag=False,
        enable_tts=False,
        memory_file=str(tmp_path / "memory.json"),
    )
    try:
        assert len(reloaded.task_memory) == 1
        assert reloaded.task_memory[0].tool_name == "framing hammer"
    finally:
        reloaded.cleanup()
