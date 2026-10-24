#!/bin/bash
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
python - <<'CHECK'
import rosgraph
import sys
if rosgraph.is_master_online():
    sys.exit('Stop the old ROS groups and ROS master before starting 01_core.sh.')
CHECK
# roslaunch creates the ROS master in this group; no separate roscore terminal.
exec roslaunch vio_homing core.launch enable_qgc_link:=true qgc_bind_ip:="$ROS_IP" qgc_peer_ip:="${VIO_QGC_IP:-192.168.50.213}" auto_frd_alignment:=true enable_homing_commands:=true "$@"
