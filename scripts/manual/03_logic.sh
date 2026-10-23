#!/bin/bash
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
manual_check /wifi_link_monitor /trajectory_recorder /homing_manager
exec roslaunch --wait vio_homing manual_logic.launch link_source:=qgc link_ok_timeout:=2.0 link_lost_timeout:=4.0 "$@"
