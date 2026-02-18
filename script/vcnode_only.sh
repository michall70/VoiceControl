#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"

source "$SCRIPT_DIR/Setup_ROS2Offboard.sh"

bash -c "
cd $WORKSPACE_DIR &&
colcon build --symlink-install --packages-select VoiceControl
"
source $WORKSPACE_DIR/install/setup.sh &&
echo '***colcon build Ready***'

# ros2 run VoiceControl VoiceControl
