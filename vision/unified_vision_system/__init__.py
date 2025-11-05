"""Unified Vision System package.

This namespace now exposes subpackages for calibration, control,
perception, human-robot interaction, metrics, and core system logic.

IMPORTANT: This package uses lazy imports to avoid dependency hell.
Import components directly from their submodules:
    
    from unified_vision_system.control.UR30Kinematics import UR30Kinematics
    from unified_vision_system.perception.OWLViTDetector import OWLViTDetector
    
Rather than importing from the top-level package.
"""

# DO NOT import anything here at module level!
# This prevents forcing all dependencies (ROS2, PyTorch, Whisper, etc.)
# to be loaded when the package is imported.

def __getattr__(name):
    """Lazy-load components on demand to avoid dependency hell."""
    
    # Only import when actually accessed
    if name == "UnifiedVisionSystem":
        from .system.UnifiedVisionSystem import UnifiedVisionSystem
        return UnifiedVisionSystem
    elif name == "UnifiedVisionSystemSim":
        from .system.UnifiedVisionSystemSim import UnifiedVisionSystemSim
        return UnifiedVisionSystemSim
    elif name == "VLMKinematicsController":
        from .control.HybridIKWrapper import VLMKinematicsController
        return VLMKinematicsController
    elif name == "UR30Kinematics":
        from .control.UR30Kinematics import UR30Kinematics
        return UR30Kinematics
    elif name == "HybridUR30Kinematics":
        from .control.UR30Kinematics import HybridUR30Kinematics
        return HybridUR30Kinematics
    
    raise AttributeError(f"module '{__name__}' has no attribute '{name}'")


__all__ = [
    "UnifiedVisionSystem",
    "UnifiedVisionSystemSim",
    "VLMKinematicsController",
    "UR30Kinematics",
    "HybridUR30Kinematics",
]
