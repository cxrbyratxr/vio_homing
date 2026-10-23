#!/bin/bash
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
manual_check /vins_estimator /vins_odometry_adapter
exec roslaunch --wait vio_homing vins.launch enable_adapter:=true "$@"
