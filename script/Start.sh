#!/bin/bash
#PATH要写死，如果source ros2 会改变PATH
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

env -i \
 TERM="$TERM" \
 HOME="$HOME" \
 PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
 USER="$USER" \
 DISPLAY="$DISPLAY" \
 XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
  DBUS_SESSION_BUS_ADDRESS="$DBUS_SESSION_BUS_ADDRESS" \
 bash --login << EOF
gnome-terminal --window --title='PX4 SITL' -- bash -lc "
source $SCRIPT_DIR/PX4_uXRCE_QGC.sh
exec bash
"
EOF
