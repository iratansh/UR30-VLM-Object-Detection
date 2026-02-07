import sys
from pathlib import Path
from unittest.mock import patch

import pytest

ROOT = Path(__file__).resolve().parents[2]
VISION = ROOT / "vision"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(VISION) not in sys.path:
    sys.path.insert(0, str(VISION))

import EnhancedConstructionRAG as rag_module  # noqa: E402
from ConstructionClarificationManager import (  # noqa: E402
    ClarificationStrategy,
    ConstructionClarificationManager,
)
from ConstructionHaystackNLP import ConstructionHaystackNLP  # noqa: E402
from EnhancedConstructionRAG import EnhancedConstructionRAG  # noqa: E402


class DummyTTS:
    def __init__(self):
        self.spoken = []

    def speak_clarification(self, text, priority=None, voice_profile=None, blocking=False):
        self.spoken.append(text)
        return True

    def cleanup(self):
        pass


def test_multimodal_clarification_flow(tmp_path):
    original_chroma = rag_module.CHROMADB_AVAILABLE
    original_sentence = rag_module.SENTENCE_TRANSFORMERS_AVAILABLE
    rag_module.CHROMADB_AVAILABLE = False
    rag_module.SENTENCE_TRANSFORMERS_AVAILABLE = False

    rag = EnhancedConstructionRAG(db_path=str(tmp_path / "rag_db"))
    dummy_tts = DummyTTS()

    try:
        with patch(
            "SharedRAGManager.get_shared_rag_manager",
            return_value=rag,
        ), patch(
            "SharedTTSManager.get_shared_tts_manager",
            return_value=dummy_tts,
        ):
            manager = ConstructionClarificationManager(
                enable_rag=True,
                enable_tts=True,
                memory_file=str(tmp_path / "memory.json"),
            )

            nlp = ConstructionHaystackNLP(confidence_threshold=0.55)
            parsed = nlp.parse_construction_command(
                "Grab the framing hammer near the sawdust pile"
            )
            assert parsed.intent == "pickup_tool"
            assert any(entity["value"] == "hammer" for entity in parsed.entities)

            detections = [
                {
                    "trade_term": "framing hammer",
                    "label": "framing hammer",
                    "category": "striking_tool",
                }
            ]
            response = manager.request_clarification(
                tool_request="hammer",
                detected_objects=detections,
                confidence_scores=[0.78],
                strategy=ClarificationStrategy.HISTORY_AWARE,
            )

            assert response.metadata.get("rag_enhanced") is True
            assert "framing hammer" in response.text.lower()

            assert dummy_tts.speak_clarification(response.text) is True
            assert dummy_tts.spoken

            contextual = rag.process_asr_for_object_detection(
                spoken_command="Grab the sawzall near the 2x4",
                user_expertise="apprentice",
            )
            assert "reciprocating saw" in contextual["target_objects_for_owlvit"]
            assert contextual["expertise_context"] == "apprentice"

            manager.cleanup()
    finally:
        rag.cleanup()
        rag_module.CHROMADB_AVAILABLE = original_chroma
        rag_module.SENTENCE_TRANSFORMERS_AVAILABLE = original_sentence
