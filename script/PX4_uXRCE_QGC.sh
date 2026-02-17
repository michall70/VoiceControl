#!/bin/bash
QGC_PATH="/home/michall/QGroundControl-x86_64.AppImage"     #edit here if your QGroundControl is in a different location
PX4_PATH=$HOME/PX4-Autopilot    #edit here if your PX4-Autopilot is in a different location

gnome-terminal --tab --title='MicroXRCEAgent udp4' -- bash -lc 'MicroXRCEAgent udp4 -p 8888'

gnome-terminal --tab --title='QGroundControl' -- bash -lc "$QGC_PATH"

cd $PX4_PATH && make px4_sitl gz_x500
