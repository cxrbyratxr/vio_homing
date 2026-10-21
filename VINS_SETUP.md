# VINS-Fusion: preliminary Jetson configuration

Standalone estimator configuration for USBGS1200P01, 640x480 YUYV capture,
ROS /camera/image_raw (bgr8) and /pixhawk/imu (sensor_msgs/Imu, imu_link FLU).
Requires the already built ROS package `vins` in the catkin workspace.

## Calibration and conventions

- PINHOLE/radtan intrinsics copied from camera_intrinsics-camchain.yaml.
- `body_T_cam0` = inverse of Kalibr `T_cam_imu`: it maps camera optical
  coordinates into ROS imu_link FLU, not Pixhawk's raw FRD axes.
- VINS source estimator.cpp uses `curTime = feature.first + td`.
  Kalibr uses `t_imu = t_cam + shift`, hence td=-0.03275171186939631 s.
- Camera timestamps must remain unchanged from calibration. Applying this
  correction in both the camera driver and VINS would double-correct time.
- Extrinsics and td are fixed during initial checks (estimate_extrinsic=0,
  estimate_td=0), to evaluate the supplied calibration without online changes.
- IMU noise values retain VINS' EuRoC example defaults (0.1, 0.01, 0.001,
  0.0001), explicitly provisional. Kalibr densities are not copied directly
  into VINS' discrete preintegration noise matrix.
- Calibration is preliminary: full Kalibr 30-iteration limit reached,
  some image residuals up to 9 px, extreme image edges underrepresented,
  and physical camera/IMU distance (~9.3 cm) not independently measured.

## Live start on Jetson

The wireless service now starts camera, UART, VINS and adapter together:

```bash
sudo systemctl restart vio-wireless.service
```

Do not start another vins.launch or adapter alongside that service. Move the
rigid camera/IMU assembly gently before a textured scene for initialization.
The current autonomous control integration is documented in AUTONOMOUS_HOMING.md.
For a standalone estimator, first stop the service and supply camera/IMU topics
on an isolated ROS master. The standalone vins.launch default still leaves the
adapter disabled.

Outputs advertised by the actual installed visualization.cpp:
- /vins_estimator/odometry: nav_msgs/Odometry (optimized IMU pose)
- /vins_estimator/imu_propagate: nav_msgs/Odometry (propagated estimate)
- /vins_estimator/path: nav_msgs/Path

The source labels odometry parent AND child `world`, while pose is of IMU
and twist linear velocity is expressed in world coordinates. Treat this raw
interface accordingly; it needs a separate audited adapter before publishing
/vio/odom_frd. World is gravity aligned with arbitrary yaw; it is not a
geographically north-aligned frame. Do not merely rename a topic or frame.
The adapter connects this raw output to /vio/odom_frd for MAVLink, homing_manager
and trajectory_recorder in the wireless service.

## Isolated bag evaluation (separate ROS master)

Terminal 1 on Jetson:

```bash
source ~/catkin_ws/devel/setup.bash
unset ROS_HOSTNAME
export ROS_IP=192.168.50.1 ROS_MASTER_URI=http://localhost:11321
mkdir -p ~/vio_data/vins/pose_graph
roslaunch -p 11321 vio_homing vins.launch \
  image_topic:=/vins/input/image_raw imu_topic:=/vins/input/imu
```

Terminal 2 (same separate master):

```bash
source ~/catkin_ws/devel/setup.bash
unset ROS_HOSTNAME
export ROS_IP=192.168.50.1 ROS_MASTER_URI=http://localhost:11321
rosbag play -r 0.5 ~/vio_data/calibration/camera_imu_motion.bag \
 /camera/image_raw:=/vins/input/image_raw /pixhawk/imu:=/vins/input/imu
```

Keep hardware data on master 11311; do not replay recorded sensor topics into
that master. No simulated-clock change is needed for this estimator's
message-based integration. Each run overwrites ~/vio_data/vins/vio.csv;
copy the output before the next run if needed.

A configuration load test alone does not validate VIO accuracy, scale,
latency, drift, or initialization. Independent motion/return-to-start tests
and coordinate adaptation remain before downstream use.

## Adapter to existing homing interface

For a standalone setup without the wireless service:

```bash
roslaunch vio_homing vins_adapter.launch
```

The existing companion nodes already subscribe to /vio/odom_frd: recorder,
homing_manager AND MAVLink bridge. Enabling this adapter therefore also sends
ODOMETRY to PX4; it does not enable homing setpoints, LAND, EKF fusion or ARM.
The adapter is not added to automatic startup.

Explicit convention, C=diag(1,-1,-1):
- p_frd = C p_vins_world (origin unchanged, arbitrary yaw, NOT geographic NED).
- R_local_frd_body_frd = C R_world_imu_flu C.
- v_body_frd = C R_world_imu_flu^T v_world.
- Angular velocity is a finite difference of optimized orientations, expressed
  in body FRD; it is not an independent gyro estimate. First pose is withheld.
- Output timestamp = VINS header timestamp + td: installed estimator integrates
  to feature timestamp + td but labels output with feature timestamp. This
  labels the state time, not a second correction of the input measurements.
- Pose covariance is rotated by blockdiag(C,C). VINS zero covariances remain
  unspecified; the existing bridge maps all-zero covariance to MAVLink unknown.
  Twist covariance stays unspecified because no angular covariance is supplied.

Output frame IDs local_frd/body_frd. Status is the String topic
/vio_homing/vins_adapter_status. Current labels world/world from this installed
VINS are checked explicitly. Nonfinite values, non-unit quaternions, stale/future
poses, nonmonotonic timestamp/sequence, >1 m position step, >1 s interruption,
or explicit /vins_restart trigger a latched FAULT. Publishing then stops.
Do not automatically restart an adapter into an old route: stop/reset the route
consumers and reinitialize the estimator before starting a new session.
This cannot detect every smooth drift or every estimator reset; no metric
accuracy claim is made. The previous 2.37 m loop error remains unresolved.
Source world is gravity aligned with arbitrary heading; axis conversion alone
does not establish north alignment for PX4 LOCAL_NED setpoints.

Alternatively start VINS and adapter together (stop previous VINS first):

```bash
roslaunch vio_homing vins.launch enable_adapter:=true
```

If changing VINS td, also set `adapter_stamp_offset` to the same value.
The default standalone `roslaunch vio_homing vins.launch` still does not
start the adapter. Existing wireless service continues to own camera/UART.

## Adapter verification on 21 October 2026

Jetson catkin_make --pkg vio_homing -j1 succeeded. Three frame-math tests passed.
Isolated synthetic ROS check recorded 8 trajectory points, 5 return points,
entered RETURN, and confirmed a latched FAULT blocks stale data and later poses.
Isolated real camera_imu_motion.bag playback (0.5 speed, /clock, no PX4 bridge)
produced 867 VINS poses, 866 FRD poses, 60 trajectory points and 16 return points;
homing states NORMAL -> LINK_LOST -> RETURN. Adapter remained ACTIVE.
The first pose is deliberately withheld to establish angular-rate history.
Evidence: verification/vins_adapter/. Test nodes stopped afterward. At that earlier verification point the live
wireless stack had enable_homing_commands=false and no live VINS/adapter.
The subsequent control integration enables both in wireless.launch.
These checks validate interfaces, not scale, drift, accuracy or flight readiness.
