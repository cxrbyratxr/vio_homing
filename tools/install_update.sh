#!/bin/bash
# Run from an unpacked deployment package. No network access required.
set -euo pipefail
package_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$HOME/catkin_ws/src/vio_homing"
expected="$(cat "$package_dir/VERSION")"
[[ "$expected" =~ ^[0-9a-f]{40}$ ]] || exit 1
if systemctl is-active --quiet vio-homing.service; then
    printf '%s\n' 'Stop vio-homing.service and manual ROS/camera/video processes first.' >&2
    exit 1
fi
if pgrep -f '([r]oslaunch|[r]osmaster|[u]sb_camera_node.py|[w]eb_video_server|[m]avlink_imu_bridge.py)' >/dev/null; then
    printf '%s\n' 'Old ROS/camera/video processes remain. Stop them before updating.' >&2
    exit 1
fi
cd "$package_dir"
sha256sum -c SHA256SUMS
cd "$repo"
git bundle verify "$package_dir/vio_homing_update.bundle"
backup_dir="$HOME/vio_backups/$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$backup_dir"
tar -czf "$backup_dir/vio_homing_before_update.tgz" -C "$HOME/catkin_ws/src" vio_homing
if [ -n "$(git status --porcelain)" ]; then
    git stash push -u -m "preserved before wireless update $backup_dir"
fi
git fetch "$package_dir/vio_homing_update.bundle" refs/heads/integrate/jetson-github
[ "$(git rev-parse FETCH_HEAD)" = "$expected" ]
git merge --ff-only FETCH_HEAD
[ "$(git rev-parse HEAD)" = "$expected" ]
# Change network variables only for this build; user's shell config unchanged.
set +u
source /opt/ros/melodic/setup.bash
unset ROS_HOSTNAME
export ROS_IP=192.168.50.1
export ROS_MASTER_URI=http://localhost:11311
cd "$HOME/catkin_ws"
catkin_make -j1
set -u
printf '\nUpdated: %s\nBackup: %s\n' "$expected" "$backup_dir"
printf '%s\n' 'Build completed. No service or PX4 parameters changed.'
printf '%s\n' 'Start: ~/catkin_ws/src/vio_homing/scripts/run_wireless.sh'
