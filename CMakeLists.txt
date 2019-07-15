cmake_minimum_required(VERSION 2.8.3)
project(elevator_navigation)

find_package(catkin REQUIRED COMPONENTS
    rospy
    ropod_ros_msgs
    maneuver_navigation
    door_status_detection
    floor_detection
    map_switcher
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    ropod_ros_msgs
    maneuver_navigation
    door_status_detection
    floor_detection
    map_switcher
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  ros/scripts/elevator_navigation
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
