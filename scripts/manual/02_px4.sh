#!/bin/bash
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
manual_check /mavlink_imu_bridge
exec roslaunch --wait vio_homing manual_px4.launch baudrate:=460800 imu_rate_hz:=200 enable_qgc_link:=true qgc_bind_ip:="$ROS_IP" qgc_peer_ip:="${VIO_QGC_IP:-192.168.50.213}" homing_execution_mode:=flight auto_frd_alignment:=true enable_homing_commands:=true "$@"
