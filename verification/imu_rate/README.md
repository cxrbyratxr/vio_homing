# IMU rate diagnosis — 2026-10-21

## Fault isolation

With IMU_INTEG_RATE=200, bridge requested HIGHRES_IMU200 but ROS measured145..147Hz.
Service was stopped to guarantee a single UART reader. A minimal pymavlink reader
at460800, without ROS/camera/video/QGC relay, also measured146.85Hz from PX4
sample timestamps (1469 samples/10s). Therefore ROS and camera processing were
not the cause of the missing rate.

PX4 console:IMU_INTEG_RATE200;uorb top reported vehicle_imu0~198Hz,
vehicle_imu1~202Hz. MAVLink stream status listed200 configured/current, but
actual transmission~147Hz. The status rate is the configured rate multiplied
by budget scaling, not an independent count of transmitted packets.
MAVLink tx_rate_mult1,txerr0,max30000B/s,UART460800.

Other message requests with integration200 did not solve the issue:
-100 requested→100.03Hz measured;
-200→146.85Hz;
-250→136.81Hz;
-300→158.58Hz;
-400→166.38Hz;bandwidth scaling started (~377 configured current).
An attempted CLI -r -1 was rejected with usage by this firmware and made no change.

## Correction

Changed only PX4 IMU_INTEG_RATE from200 to400, saved and rebooted Pixhawk.
Kept UART460800,MAV_1_RATE30000 and companion HIGHRES_IMU request200 unchanged.
Readback after reboot confirmed saved400. No ARM,EKF/failsafe parameter changes,
odometry or calibration changes. Source400 is a nominal integration setting;
uORB instances observed~334 and404Hz after the change, so exact source cadence
is device dependent. Minimal UART reader then measured199.36Hz (3988 samples/20s).

PX4 HIGHRES_IMU sends only when a selected vehicle_imu subscription updates.
Its independent MAVLink scheduler and source cadence can miss updates when
configured at similar rates; faster integration provides fresh samples at each
200Hz transmission opportunity. This is the mechanism supported by source code
and the controlled before/after observation, not a ROS resampling workaround.

Samples remain authentic with original PX4 sample timestamps. Intervals are
not exactly5ms:raw corrected stream used about3/6ms with rare9ms intervals.
Averaging to200Hz must not be confused with perfectly uniform sampling.

Source:[HIGHRES_IMU](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/modules/mavlink/streams/HIGHRES_IMU.hpp),
[scheduler](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/modules/mavlink/mavlink_stream.cpp),
[IMU_INTEG_RATE](https://docs.px4.io/v1.16/en/advanced_config/parameter_reference#IMU_INTEG_RATE).

## Full stack acceptance (60 seconds)

Camera/video/QGC relay/ROS active, service running and QGC LINK_OK:
-IMU11980 messages,stamp199.5467Hz,arrival199.5475Hz;
-dt2.945..9.043ms,std1.438ms,no nonpositive stamps,no gaps>15ms;
-camera1800 images,30.0034Hz,dt31.331..36.572ms;
-TIMESYNC OK,ACK0,no connection-loss entries after final start14:03:12MSK.
Raw result:result.json. The generic gaps_gt_15ms field is only meaningful for
IMU; camera30Hz naturally has every interval greater than15ms.

No synthetic odometry or calibration run was started. Script temporarily opened
UART only while the normal service was stopped; service restored after each sweep.
No changes to Jetson bridge code were needed. Corrected PX4 parameter is persistent.
Reference parameter file:config/px4_imu_stream.params. It is documentation for
this bench, not an automatically applied parameter set for other hardware.
Rollback if needed:restore IMU_INTEG_RATE200 via PX4 console,save,reboot; the
previous~145Hz behavior may recur. Bridge still requests200 after reconnect.
