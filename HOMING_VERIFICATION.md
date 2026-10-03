# Jetson verification

Tested directly in `/home/jetson/catkin_ws/src/vio_homing`, using ROS Melodic
1.14.13 and Python 2.7.17.

## Results

- `catkin_make -j2`: passed.
- `catkin_make run_tests_vio_homing -j1`: passed.
- `catkin_test_results build/test_results/vio_homing`: **14 tests, 0 errors,
  0 failures, 0 skipped**. This count includes ROS test runner results;
  the suite contains 10 logic tests and two ROS scenarios.
- `roslaunch vio_homing homing_stand.launch` on master port 11321: passed,
  including simulator completion and clean shutdown.

The interactive stand's final episode used the real monitor and recorder:

| Measurement | Observed |
|---|---:|
| Heartbeat stop to RETURN | 3.125 s |
| Confirmed LINK_LOST state to RETURN state | 0.021 s |
| Existing return_path points | 6 |
| WAIT_LINK to land_command=true | 60.049 s |

Checked ordered goal delivery, arrival at the frozen last-good endpoint,
recovery during RETURN, recovery during WAIT_LINK, LAND intent, and recovery
after LAND. The ROS interface test additionally checked callback ordering,
stale latched paths, empty paths, endpoint/frame mismatches, stale and NaN
odometry, and a singleton return path.

## Evidence on Jetson

- `/tmp/vio_homing_stand_result.json`: interactive stand measurements and
  state transitions.
- `/tmp/vio_homing_rostest_result.json`: end-to-end rostest result.
- `/tmp/vio_homing_launch.log`: interactive launch output and PASS messages.
- `/tmp/vio_homing_tests_final.log`: catkin test execution.
- `/home/jetson/catkin_ws/build/test_results/vio_homing/`: test XML results.

The standalone test masters and UDP ports were separate from the running
companion. The original companion nodes remained running on port 11311.
SHA-256 comparison confirmed that `wifi_link_monitor.py`,
`trajectory_recorder.py`, and `mavlink_imu_bridge.py` retained their original
contents. The user's existing `check_imu_timing.py` changes were preserved.

The manager is added to `companion.launch` for its next launch. Its outputs
currently express ROS goals and LAND intent; the bridge has no subscriber
that turns these into PX4 flight commands. These tests validate the bench
state machine and ROS interfaces, not physical aircraft landing.

See `HOMING_STAND.md` for exact build, test and stand commands.


PX4 hardware transport verification added in PX4_HOMING_BENCH.md and verification/px4_bench/. The ROS-only stage above is historical.
