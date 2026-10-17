# Wireless service (Jetson only)

Requires already deployed wireless.launch/run_wireless.sh (390e31d or newer),
web_video_server, existing AP with antennas, PX4 TELEM2=460800/MAV_1_RATE=30000.
Fixed current Mac IP192.168.50.24; update unit if DHCP changes. No automatic IP
learning. No AP password stored. No ARM or homing commands enabled.

Stop recording cleanly and stop manual stack/camera/video with Ctrl-C first.
Stop services before installing:

```bash
sudo systemctl stop vio-homing.service vio-wireless.service
```

A not-found error for the new unit on first install is expected; ensure the old
unit is actually stopped. Enable AP autoconnect without disconnecting it:

```bash
sudo nmcli connection modify VIO-HOMING connection.autoconnect yes
```

Install using `sudo bash tools/install_wireless_service.sh` from repo, or the
same installer from the standalone service package. It backs up old unit/state,
disables the old service, installs/enables/starts vio-wireless.service.
It refuses existing ROS processes and does not kill them. No catkin rebuild.
No ~/.bashrc or PX4 parameter changes. Existing systemd drop-ins for a previously
installed vio-wireless unit should be reviewed with systemctl cat if applicable.

```bash
systemctl is-enabled vio-wireless.service
systemctl status vio-wireless.service --no-pager -l
journalctl -u vio-wireless.service -n 60 --no-pager
```

Expect active launch, PX4 connected, TIMESYNC OK, IMU interval ACK0, camera~30Hz,
QGC LINK_OK after QGC connects. Empty CameraInfo is expected before calibration.
Systemd active alone does not prove every respawning ROS node healthy.

Service owns the whole process group. SIGINT permits roslaunch shutdown, then
systemd cleans up remaining children. Failed launch/absent AP retries after10s.
Don't launch another companion or camera manually. SSH disconnect no longer
owns the stack. Use another SSH terminal for checks/bag recording, with source
workspace, unset ROS_HOSTNAME, ROS_IP=192.168.50.1,ROS_MASTER_URI=http://localhost:11311.

First prove survival by closing/reopening SSH without reboot. Then perform a
controlled reboot after closing any bag writer and verify AP, service, QGC,
video and rates. Boot readiness on hardware has not been verified locally.

Rollback:stop+disable vio-wireless.service. The old service defaults may still
be115200; don't enable it against PX4460800 without fixing its configuration.
Run the known manual run_wireless.sh after the new service fully stops if needed.
