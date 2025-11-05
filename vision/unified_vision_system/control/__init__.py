"""Control subpackage for unified_vision_system.

LAZY IMPORTS: Import components directly to avoid dependency hell.
Example: from unified_vision_system.control.UR30Kinematics import UR30Kinematics
"""

# DO NOT import at module level - use lazy loading via __getattr__

def __getattr__(name):
    """Lazy-load control components."""
    if name == "VLMKinematicsController":
        from .HybridIKWrapper import VLMKinematicsController
        return VLMKinematicsController
    elif name == "HybridUR30Kinematics":
        from .UR30Kinematics import HybridUR30Kinematics
        return HybridUR30Kinematics
    elif name == "UR30Kinematics":
        from .UR30Kinematics import UR30Kinematics
        return UR30Kinematics
    elif name == "UR30GraspController":
        from .ur30_grasp_controller import UR30GraspController
        return UR30GraspController
    raise AttributeError(f"module '{__name__}' has no attribute '{name}'")

__all__ = [
    "HybridUR30Kinematics",
    "UR30Kinematics",
    "VLMKinematicsController",
    "UR30GraspController",
]
