#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"

gnome-terminal --tab --title='MicroXRCEAgent udp4' -- bash -lc 'MicroXRCEAgent udp4 -p 8888'

gnome-terminal --tab --title='QGroundControl' -- bash -lc "$QGC_PATH"

cd $PX4_PATH && make px4_sitl gz_x500
