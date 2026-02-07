#!/bin/bash
# Load controllers and immediately move UR30 to the observation pose so the eye-in-hand camera
# starts with a clear view instead of being occluded at the all-zero home position.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Loading controllers..."
"${SCRIPT_DIR}/spawn_controllers.sh"

echo "Sending observation pose trajectory so the camera can see the workspace..."
"${SCRIPT_DIR}/set_observation_pose.sh"
