#!/bin/bash
# One foreground stack; override inherited Ethernet ROS environment.
set -eo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="${VIO_CATKIN_WS:-$HOME/catkin_ws}"
if [ "${1:-}" = "--help" ]; then
    printf '%s\n' 'Usage: run_wireless.sh [roslaunch name:=value ...]' \
      'Defaults: Jetson 192.168.50.1, Mac 192.168.50.24, UART 460800, IMU200.' \
      'VIO_JETSON_IP overrides both ROS_IP and relay/video bind address.'
    exit 0
fi
source /opt/ros/melodic/setup.bash
source "$workspace/devel/setup.bash"
unset ROS_HOSTNAME
export ROS_IP="${VIO_JETSON_IP:-192.168.50.1}"
export ROS_MASTER_URI=http://localhost:11311
if ! ip -4 -o addr show | awk '{print $4}' | cut -d/ -f1 | grep -Fxq "$ROS_IP"; then
    printf 'AP address %s is not assigned. Start VIO-HOMING with antennas connected.\n' "$ROS_IP" >&2
    exit 1
fi
if systemctl is-active --quiet vio-homing.service; then
    printf '%s\n' 'Stop vio-homing.service before starting this stack.' >&2
    exit 1
fi
for argument in "$@"; do
    case "$argument" in
        jetson_ip:=*) printf '%s\n' 'Use VIO_JETSON_IP to change ROS and bind IP together.' >&2; exit 1 ;;
    esac
done
# Fail before roslaunch could replace nodes with identical names.
python - <<'CHECK'
import socket
import sys
import xmlrpclib
try:
    socket.setdefaulttimeout(2)
    code, message, pid = xmlrpclib.ServerProxy(
        'http://localhost:11311').getPid('/wireless_preflight')
except Exception:
    pass
else:
    sys.exit('ROS master already running; stop the old stack first.')
for kind, port in ((socket.SOCK_DGRAM, 14555), (socket.SOCK_STREAM, 8080)):
    sock = socket.socket(socket.AF_INET, kind)
    try:
        sock.bind(('', port))
    except socket.error:
        sys.exit('Port %d already occupied; stop the old relay/video first.' % port)
    finally:
        sock.close()
CHECK
exec roslaunch vio_homing wireless.launch "jetson_ip:=$ROS_IP" "$@"
