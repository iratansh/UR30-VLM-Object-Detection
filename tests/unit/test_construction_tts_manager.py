import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
VISION = ROOT / "vision"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(VISION) not in sys.path:
    sys.path.insert(0, str(VISION))

from ConstructionTTSManager import (  # noqa: E402
    ConstructionTTSManager,
    TTSPriority,
    VoiceProfile,
)


def test_mock_tts_speaks_clarifications_blocking():
    manager = ConstructionTTSManager(
        voice_profile=VoiceProfile.PROFESSIONAL,
        enable_background_speech=False,
        construction_mode=True,
        use_coqui=False,
    )
    try:
        success = manager.speak_clarification(
            "Testing mock TTS", priority=TTSPriority.NORMAL, blocking=True
        )
        assert success is True
    finally:
        manager.cleanup()


def test_construction_pronunciations_are_applied():
    manager = ConstructionTTSManager(
        voice_profile=VoiceProfile.PROFESSIONAL,
        enable_background_speech=False,
        construction_mode=True,
        use_coqui=False,
    )
    try:
        processed = manager._apply_construction_pronunciations("Grab the sawzall and 2x4")
        assert "saw-zall" in processed
        assert "two by four" in processed.lower()
    finally:
        manager.cleanup()


def test_speech_capabilities_report_contains_core_fields():
    manager = ConstructionTTSManager(
        voice_profile=VoiceProfile.PROFESSIONAL,
        enable_background_speech=False,
        construction_mode=True,
        use_coqui=False,
    )
    try:
        status = manager.test_speech_capabilities()
        for key in {
            "engine_available",
            "background_speech_enabled",
            "current_profile",
        }:
            assert key in status
        assert status["current_profile"] == VoiceProfile.PROFESSIONAL.value
    finally:
        manager.cleanup()
