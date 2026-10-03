# VIO Homing: Jetson bench scenario

The implementation is compatible with ROS Melodic / Python 2.7. The existing
Wi-Fi monitor, trajectory recorder and MAVLink bridge keep their interfaces.

## Existing inputs

| Topic | Type | Use |
|---|---|---|
| `/vio/odom_frd` | `nav_msgs/Odometry` | Current local FRD position |
| `/vio_homing/link_state` | `std_msgs/String` | `WAITING`, `LINK_OK`, `LINK_DEGRADED`, `LINK_LOST` |
| `/vio_homing/return_path` | `nav_msgs/Path` | Recorder's already-built route, in return order |
| `/vio_homing/return_ready` | `std_msgs/Bool` | Recorder has a route available |
| `/vio_homing/last_good_link_pose` | `geometry_msgs/PoseStamped` | Frozen endpoint for the return episode |

`~odom_topic` can override the odometry input, including through `companion.launch`.

## New outputs

| Topic | Type | Meaning |
|---|---|---|
| `/vio_homing/homing_state` | `std_msgs/String` | `NORMAL`, `LINK_LOST`, `RETURN`, `WAIT_LINK`, `LAND` |
| `/vio_homing/return_goal` | `geometry_msgs/PoseStamped` | Current goal at 20 Hz; nonlatched |
| `/vio_homing/return_goal_index` | `std_msgs/UInt32` | Zero-based index in the frozen route |
| `/vio_homing/return_active` | `std_msgs/Bool` | True only during `RETURN` and `WAIT_LINK` |
| `/vio_homing/land_command` | `std_msgs/Bool` | True in `LAND`; false on recovery |

State, active and land intent are latched. A consumer must honor
`return_active=false` to discard a previously received goal. Goal positions and
orientations come from the existing path; their timestamps are refreshed.
These outputs are bench interfaces. The current MAVLink bridge only transports
IMU/odometry/time synchronization and does not execute these goals or LAND.
No PX4 flight command is sent by this implementation.

## State behavior

1. Startup `WAITING` remains `NORMAL`. At least one observed `LINK_OK` is
   required before loss can activate return.
2. `LINK_DEGRADED` does not initiate return. The monitor confirms `LINK_LOST`
   after its existing 3-second UDP timeout.
3. `LINK_LOST` waits for ready=true, a nonempty valid path, and a matching
   last-good endpoint. A missing/invalid route never causes immediate LAND.
4. The manager snapshots the existing path, accepts path/ready in either
   callback order (including before loss), and rejects paths older than its
   latest healthy-link observation. It does not rebuild the path.
5. Every point is issued in path order. Advancement requires fresh, finite,
   same-frame odometry within `~arrival_radius` (default 0.4 m). Endpoint
   arrival also checks the frozen `last_good_link_pose`.
6. `WAIT_LINK` holds the endpoint. `~wait_link_seconds` defaults to 60 and
   cannot be configured below 60. The count uses ROS time and requires
   continuous fresh odometry at the endpoint. Stale odometry, drift outside
   the radius, or a backward clock jump restarts the count.
7. The next state is `LAND`, with land_command=true and return_active=false.
8. `LINK_OK` at any phase cancels return, stops goal publication, clears the
   active route, resets land intent, and returns to `NORMAL`. A later loss
   needs a fresh recorder route. `LINK_DEGRADED` alone does not cancel return.

`~odom_timeout` defaults to 1 second and checks both message stamp and reception
age. Frames are checked without implicit coordinate conversion or TF lookup.

## Build and run tests on Jetson

```bash
source /opt/ros/melodic/setup.bash
cd ~/catkin_ws
catkin_make -j2
source devel/setup.bash
ROS_MASTER_URI=http://127.0.0.1:11321 catkin_make run_tests_vio_homing -j1
catkin_test_results build/test_results/vio_homing
```

The test target runs Python 2.7 unit tests, ROS transport tests, and a full
end-to-end stand using the existing UDP monitor and recorder. Rostest creates
its own master. The end-to-end UDP monitor uses localhost port 15052.
The test sends real heartbeats, stops them, moves synthetic odometry during
degradation, follows the recorded goals, and maintains fresh pose during the
full 60-second wait. It also checks recovery during RETURN and WAIT_LINK and
after LAND. Allow approximately 90 seconds for the tests after configuration.

Full test JSON: `/tmp/vio_homing_rostest_result.json`.
JUnit/nose results: `~/catkin_ws/build/test_results/vio_homing/`.

## Interactive stand

Use a separate master so synthetic odometry does not reach the running
companion's MAVLink bridge. In a Jetson terminal:

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11321
unset ROS_IP ROS_HOSTNAME
roslaunch vio_homing homing_stand.launch
```

This launch creates a separate master when none is running at that address.
It uses localhost UDP port 15051 and starts no MAVLink bridge.
Expected logs include:

```text
PASS: recovery during RETURN cancels goals
PASS: recovery during WAIT_LINK cancels LAND
PASS: full stand, WAIT_LINK=60.xxx s, path=N points
```

The final scenario is `NORMAL -> LINK_LOST -> RETURN -> WAIT_LINK -> LAND`,
followed by link recovery back to `NORMAL`. The simulator writes
`/tmp/vio_homing_stand_result.json` and exits, stopping this stand launch.

While it is running, another Jetson terminal with the same master can inspect:

```bash
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11321
rostopic echo /vio_homing/homing_state
# In other terminals with the same ROS_MASTER_URI:
rostopic echo /vio_homing/return_goal
rostopic echo /vio_homing/return_active
rostopic echo /vio_homing/land_command
```

`companion.launch` starts the manager on its next launch with the existing
odometry argument, 0.4 m radius, 1 s odometry timeout and 60 s wait.


PX4 hardware transport verification added in PX4_HOMING_BENCH.md and verification/px4_bench/. The ROS-only stage above is historical.
