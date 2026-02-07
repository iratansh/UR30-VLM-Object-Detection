import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
VISION = ROOT / "vision"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(VISION) not in sys.path:
    sys.path.insert(0, str(VISION))

import EnhancedConstructionRAG as rag_module  # noqa: E402
from SharedRAGManager import SharedRAGManager, get_shared_rag_manager  # noqa: E402
from SharedTTSManager import SharedTTSManager, get_shared_tts_manager  # noqa: E402


def test_shared_rag_manager_singleton(tmp_path):
    SharedRAGManager.reset_singleton()
    rag_module.CHROMADB_AVAILABLE = False
    rag_module.SENTENCE_TRANSFORMERS_AVAILABLE = False

    manager_one = get_shared_rag_manager()
    manager_two = get_shared_rag_manager()
    assert manager_one is not None
    assert manager_one is manager_two

    SharedRAGManager.reset_singleton()
    manager_three = get_shared_rag_manager()
    assert manager_three is not manager_one
    SharedRAGManager.reset_singleton()


def test_shared_tts_manager_singleton():
    SharedTTSManager.reset_singleton()

    tts_one = get_shared_tts_manager(
        voice_profile=None,
        enable_background_speech=False,
        construction_mode=True,
        use_coqui=False,
    )
    tts_two = get_shared_tts_manager(
        voice_profile=None,
        enable_background_speech=False,
        construction_mode=True,
        use_coqui=False,
    )
    assert tts_one is not None
    assert tts_one is tts_two

    SharedTTSManager.reset_singleton()
    tts_three = get_shared_tts_manager(
        voice_profile=None,
        enable_background_speech=False,
        construction_mode=True,
        use_coqui=False,
    )
    assert tts_three is not tts_one
    SharedTTSManager.reset_singleton()
