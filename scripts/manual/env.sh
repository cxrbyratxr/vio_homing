#!/bin/bash
# Source from the numbered foreground launchers.
set -eo pipefail
source /opt/ros/melodic/setup.bash
source "${VIO_CATKIN_WS:-$HOME/catkin_ws}/devel/setup.bash"
unset ROS_HOSTNAME
export ROS_IP="${VIO_JETSON_IP:-192.168.50.1}"
export ROS_MASTER_URI=http://localhost:11311
export ROSCONSOLE_FORMAT='[${severity}] [${time}]: ${message}'
manual_check() {
    python - "$@" <<'PY'
import sys
import rosgraph
try:
    nodes = rosgraph.Master('/manual_start_check').getSystemState()
except Exception:
    sys.exit('Start 01_core.sh first; ROS master is unavailable.')
active = set(n for group in nodes for topic, names in group for n in names)
required = set(['/mavlink_imu_bridge', '/wifi_link_monitor',
                '/trajectory_recorder', '/homing_manager'])
missing = required - active
if missing:
    sys.exit('Core group is not ready: ' + ', '.join(sorted(missing)))
conflicts = active.intersection(sys.argv[1:])
if conflicts:
    sys.exit('Already running: ' + ', '.join(sorted(conflicts)))
PY
}
