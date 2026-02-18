#!/bin/bash

#==== edit here ====================================================================================
export QGC_PATH="/home/michall/QGroundControl-x86_64.AppImage"  #QGroundControl的路径
export PX4_PATH=$HOME/PX4-Autopilot                             #PX4-Autopilot的路径

export WORKSPACE_DIR="$HOME/px4_ros_com_ws"                     #ROS2工作空间的路径
export AGENT_PACKAGES_DIR="$HOME/gemini-agent/agent-venv/lib/python3.12/site-packages"    #AI agent的Python包路径，该目录下应直接包含agent包
export AGENT_DOTENV_DIR="$HOME/gemini-agent"                    #AI agent的dotenv文件夹路径，非.env文件路径，包含内容 DASHSCOPE_API_KEY=sk-xxxxxx
#===================================================================================================
echo "***env has been loaded***"

#==== edit here ====================================================================================
export https_proxy=http://127.0.0.1:7897
export http_proxy=http://127.0.0.1:7897
#===================================================================================================
echo "***proxy Ready***"
