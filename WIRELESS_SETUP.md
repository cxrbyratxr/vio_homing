# Wi-Fi camera + QGC, 2026-10-17

## Current measured bench

Jetson AP VIO-HOMING:192.168.50.1; Mac Wi-Fi:192.168.50.24.
Camera USBGS1200P01:640x480 YUYV, color BGR8, measured30Hz after removing
CAP_PROP_BUFFERSIZE=1. Focus fixed by owner. This does not establish intrinsics.
TELEM2:PX4 /dev/ttyS3 ↔ Jetson /dev/ttyTHS1,460800 baud.
PX4 parameters already set by owner:SER_TEL2_BAUD=460800,MAV_1_RATE=30000.
HIGHRES_IMU200 requested:1000 timestamp samples mean196.17Hz,max9.919ms,
no backward stamps/no gaps>15ms. Earlier test under Kalibr build load had reconnects.
Full wireless camera+video+QGC+IMU200 acceptance must still be repeated.

## Update without internet

Mac transfer commands are in ../deploy/README.md outside the source checkout.
Installer backs up the full repository and stashes tracked/untracked changes.
It only fast-forwards, never overwrites divergent commits. Saved stashes are not
reapplied automatically: Jetson's manual camera/launch edits are superseded by
these integrated changes; unrelated edits must be reviewed from stash/backup.
No changes to ~/.bashrc, networking profiles, AP password or PX4 parameters.
The old service remains stopped until separately configured for this launch.
Kalibr workspace, bags and measured aprilgrid.yaml are outside the repo and unchanged.

## Jetson prerequisites

With internet, install if absent:

```bash
sudo apt-get install ros-melodic-web-video-server
```

Don't reinstall ROS/JetPack/OpenCV. If apt proposes removals, inspect before accepting.
AP must already be active with antennas connected. Script fails if its IP is absent.
Mac must have the configured address; DHCP may change it on reconnect.

## One launch terminal

Stop any bag recording cleanly with Ctrl-C. Stop old manual companion, camera and
web_video_server with Ctrl-C, and stop the old service:

```bash
sudo systemctl stop vio-homing.service
```

After update, connect from Mac:

```bash
ssh jetson@192.168.50.1
```

On Jetson:

```bash
~/catkin_ws/src/vio_homing/scripts/run_wireless.sh
```

All four components now start together:companion/homing monitor,camera,video,QGC relay.
No separate roscore/camera/video terminals. The launcher overrides inherited
ROS_HOSTNAME and ROS_IP after sourcing the workspace:ROS_IP=192.168.50.1,
ROS_MASTER_URI=http://localhost:11311. All ROS nodes are on Jetson; Mac needs
only browser and QGC, not ROS. localhost master is intentional, node advertised
addresses use Wi-Fi. It refuses an active old service/master or occupied ports.
It does not fix a missing AP, learn a peer IP, kill unrelated processes or ARM PX4.

If Mac DHCP changed:

```bash
~/catkin_ws/src/vio_homing/scripts/run_wireless.sh qgc_peer_ip:=192.168.50.24
```

Replace with actual Mac address. Use imu_rate_hz:=100 to fall back, or
 enable_video:=false to disable browser preview. Default camera uses observed
/dev/v4l/by-id/usb-Global_Shutter_Camera_Global_Shutter_Camera_01.00.00-video-index0;
if absent verify `ls -l /dev/v4l/by-id` and pass camera_device:=/dev/video0 only
when that node is confirmed. Grayscale remains configurable in companion.launch.
CameraInfo warning is expected until measured ROS calibration YAML is supplied.

## QGC and browser on Mac

QGC:one UDP link listening14550,target192.168.50.1:14555. Disable the old
Ethernet and USB links during proof of Wi-Fi transport. Expected GCS ID255/190.
Open http://192.168.50.1:8080/stream?topic=/camera/image_raw in browser.
The image preview is compressed for display, bag records original image topic.
Keep preview clients to one and check rates under combined load.

## A second terminal for checks and recording

```bash
ssh jetson@192.168.50.1
source ~/catkin_ws/devel/setup.bash
unset ROS_HOSTNAME
export ROS_IP=192.168.50.1
export ROS_MASTER_URI=http://localhost:11311
rosnode list
rostopic hz /camera/image_raw
# Ctrl-C after 15s
rostopic hz /pixhawk/imu
# Ctrl-C after 60s
rostopic echo /vio_homing/link_state
# Ctrl-C
```

Expect camera~30Hz,IMU~200Hz (not guaranteed exact),LINK_OK while QGC sends heartbeat,
TIMESYNC OK and no reconnects. Bridge logs SET_MESSAGE_INTERVAL ACK result0 when
accepted. ACK only confirms request acceptance; measure actual timestamps too.
200Hz requested after each UART connect,not repeatedly from another serial reader.
The bridge does not increase PX4 serial baud or rate limits itself. Never start
legacy companion defaults115200 against PX4 now460800.

For reproducible calibration, stop preview if it degrades timestamps. Don't
change focus/resolution mid-session. Camera/IMU still require full calibration.
No ARM/Takeoff/flight test is part of wireless preview acceptance.

## Service and SSH lifetime

This version is a foreground trial. Keep launch SSH terminal open:closing it or
losing Wi-Fi may terminate the stack. For a controlled disconnect test use a
local Jetson session or an installed terminal multiplexer; do not rely on a
Wi-Fi SSH shell for autonomous survival. Do not enable old service blindly:
a systemd override must be configured after inspecting `systemctl cat vio-homing.service`.
Current homing transport is DISARMED bench only and remains disabled by default.
Closing QGC tests GCS heartbeat independently of SSH and video.

After restarting capture, read `v4l2-ctl -d /dev/video0 --get-ctrl=focus_auto,focus_absolute,exposure_auto,exposure_absolute`. Focus was fixed by owner, but persistence across camera open/replug is not assumed. Do not set an unmeasured focus position from old logs.

## Optional SSH-independent autostart

See [systemd instructions](systemd/README.md). Dedicated vio-wireless.service
is now prepared; older foreground-lifetime limitations apply to manual launch.
Installation and boot verification remain to be done on Jetson.
