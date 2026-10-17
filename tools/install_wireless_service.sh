#!/bin/bash
set -euo pipefail
if [ "$(id -u)" -ne 0 ]; then
    printf '%s\n' 'Run with sudo bash install_wireless_service.sh' >&2
    exit 1
fi
package_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
unit="$package_dir/vio-wireless.service"
if [ ! -f "$unit" ]; then
    unit="$package_dir/../systemd/vio-wireless.service"
fi
[ -f "$unit" ]
[ -x /home/jetson/catkin_ws/src/vio_homing/scripts/run_wireless.sh ]
[ -f /home/jetson/catkin_ws/src/vio_homing/launch/wireless.launch ]
[ -f /home/jetson/catkin_ws/devel/setup.bash ]
if systemctl is-active --quiet vio-homing.service || systemctl is-active --quiet vio-wireless.service; then
    printf '%s\n' 'Stop both ROS services before installation.' >&2
    exit 1
fi
if pgrep -f '([r]oslaunch|[r]osmaster|[u]sb_camera_node.py|[w]eb_video_server|[m]avlink_imu_bridge.py)' >/dev/null; then
    printf '%s\n' 'Stop manual ROS/camera/video terminals with Ctrl-C first.' >&2
    exit 1
fi
backup_dir="/home/jetson/vio_backups/service_$(date +%Y%m%d_%H%M%S)_$$"
install -d -m 755 "$backup_dir"
systemctl cat vio-homing.service > "$backup_dir/old-service.txt" || true
systemctl is-enabled vio-homing.service > "$backup_dir/old-enabled.txt" || true
if [ -f /etc/systemd/system/vio-wireless.service ]; then
    cp -a /etc/systemd/system/vio-wireless.service "$backup_dir/"
fi
# Preserve the old unit for audit/rollback, but prevent concurrent autostart.
systemctl disable vio-homing.service
install -m 644 "$unit" /etc/systemd/system/vio-wireless.service
systemctl daemon-reload
systemctl enable --now vio-wireless.service
printf '\nService installed; backup: %s\n' "$backup_dir"
printf '%s\n' 'Inspect: systemctl status vio-wireless.service --no-pager -l' \
  'Logs: journalctl -u vio-wireless.service -n 60 --no-pager' \
  'Restart=always retries missing AP every 10 seconds; inspect logs for readiness.'
