cmake_minimum_required(VERSION 3.0.2)

project(vio_homing)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
  geometry_msgs
  nav_msgs
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    sensor_msgs
    cv_bridge
    geometry_msgs
    nav_msgs
    std_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/mavlink_imu_bridge.py
    scripts/check_imu_timing.py
    scripts/wifi_link_monitor.py
    scripts/trajectory_recorder.py
    scripts/homing_manager.py
    scripts/usb_camera_node.py
    scripts/check_camera_timing.py
    scripts/vins_odometry_adapter.py
    scripts/vins_displacement.py
    scripts/set_bench_origin.py
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY scripts/manual/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts/manual
  USE_SOURCE_PERMISSIONS
)
