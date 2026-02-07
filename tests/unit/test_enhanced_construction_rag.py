import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
VISION = ROOT / "vision"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(VISION) not in sys.path:
    sys.path.insert(0, str(VISION))

import EnhancedConstructionRAG as rag_module  # noqa: E402
from EnhancedConstructionRAG import EnhancedConstructionRAG  # noqa: E402


@pytest.fixture()
def construction_rag():
    rag_module.CHROMADB_AVAILABLE = False
    rag_module.SENTENCE_TRANSFORMERS_AVAILABLE = False
    rag = EnhancedConstructionRAG()
    yield rag
    rag.cleanup()


def test_retrieve_relevant_knowledge_returns_items(construction_rag):
    results = construction_rag.retrieve_relevant_knowledge(
        query="How do I use a framing hammer?",
        tool_context=["framing hammer"],
        expertise_level="apprentice",
        n_results=2,
    )
    assert results
    assert any("hammer" in item.content.lower() for item in results)


def test_enhance_clarification_adds_context(construction_rag):
    response = construction_rag.enhance_clarification(
        original_text="I found a hammer. Is this correct?",
        tool_request="hand me the hammer",
        detected_tools=[{"trade_term": "framing hammer"}],
        user_expertise="journeyman",
    )
    assert response.retrieved_items
    assert response.enhanced_text.startswith("I found a hammer")
    assert "pro tip" in response.enhanced_text.lower()


def test_process_asr_maps_construction_slang(construction_rag):
    processed = construction_rag.process_asr_for_object_detection(
        spoken_command="Grab the sawzall near the 2x4",
        user_expertise="apprentice",
    )
    targets = processed["target_objects_for_owlvit"]
    assert "reciprocating saw" in targets
    assert "2x4 lumber" in targets
    assert processed["original_command"] == "Grab the sawzall near the 2x4"


def test_cleanup_resets_backend(construction_rag):
    construction_rag.cleanup()
    assert construction_rag.collection is None
    assert construction_rag.chroma_client is None
    assert construction_rag.embedding_model is None
