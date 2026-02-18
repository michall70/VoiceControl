#!/bin/bash

#==== edit here ====================================================================================
export QGC_PATH="/home/michall/QGroundControl-x86_64.AppImage"
export PX4_PATH=$HOME/PX4-Autopilot

export WORKSPACE_DIR="$HOME/px4_ros_com_ws"        
export AGENT_PACKAGES_DIR="$HOME/gemini-agent/agent-venv/lib/python3.12/site-packages"    
export AGENT_DOTENV_DIR="$HOME/gemini-agent"    
#===================================================================================================

echo "***env has been loaded***"
