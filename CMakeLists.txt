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
    scripts/test_odometry.py
    scripts/check_imu_timing.py
    scripts/wifi_link_monitor.py
    scripts/trajectory_recorder.py
    scripts/homing_manager.py
    scripts/homing_stand_sim.py
    scripts/px4_homing_bench.py
    scripts/run_px4_homing_bench.py
    scripts/usb_camera_node.py
    scripts/check_camera_timing.py
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_install_python(
    PROGRAMS test/homing_interfaces.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  catkin_add_nosetests(test/test_qgc_link.py)
  catkin_add_nosetests(test/test_camera_calibration.py)
  catkin_add_nosetests(test/test_homing_logic.py)
  catkin_add_nosetests(test/test_homing_transport.py)
  add_rostest(test/homing_stand.test)
  add_rostest(test/homing_interfaces.test)
endif()
