#!/bin/bash
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
manual_check /usb_camera /vins_estimator /vins_odometry_adapter
mkdir -p "$HOME/vio_data/vins/pose_graph"
exec roslaunch --wait vio_homing vio.launch "$@"
