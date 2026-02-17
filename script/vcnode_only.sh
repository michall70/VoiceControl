#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$HOME/px4_ros_com_ws"        #edit here if your workspace is in a different location

source "$SCRIPT_DIR/Setup_ROS2Offboard.sh"

source "$SCRIPT_DIR/proxy.sh"

bash -c "
cd $WORKSPACE_DIR &&
colcon build --symlink-install --packages-select VoiceControl
"
source $WORKSPACE_DIR/install/setup.sh &&
echo '***colcon build Ready***'

# ros2 run VoiceControl VoiceControl
