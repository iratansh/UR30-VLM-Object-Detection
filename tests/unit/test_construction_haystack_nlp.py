import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
VISION = ROOT / "vision"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(VISION) not in sys.path:
    sys.path.insert(0, str(VISION))

from ConstructionHaystackNLP import (  # noqa: E402
    ConstructionHaystackNLP,
)


@pytest.fixture()
def nlp():
    # Force fallback implementation to avoid heavy dependencies
    ConstructionHaystackNLP.HAYSTACK_AVAILABLE = False
    return ConstructionHaystackNLP()


def test_parse_pickup_command_identifies_intent(nlp):
    result = nlp.parse_construction_command("pick up the framing hammer")
    assert result.intent == "pickup_tool"
    assert any(entity["value"] in {"hammer", "framing hammer"} for entity in result.entities)
    assert result.confidence >= 0.6


def test_empty_input_returns_error_metadata(nlp):
    result = nlp.parse_construction_command("   ")
    assert result.intent == "unknown"
    assert result.confidence == 0.0
    assert result.metadata.get("error") == "Empty input"


def test_is_valid_construction_command_uses_threshold(nlp):
    assert nlp.is_valid_construction_command("grab the drill")
    assert not nlp.is_valid_construction_command("sing a song")


def test_add_training_examples_extends_corpus(nlp):
    before = len(nlp.training_examples)
    nlp.add_training_examples([
        {"text": "secure the ladder", "intent": "pickup_tool", "entities": []}
    ])
    assert len(nlp.training_examples) == before + 1


def test_answer_construction_question_without_backend(nlp):
    answer = nlp.answer_construction_question("What is a framing hammer used for?")
    assert "not available" in answer.lower()
