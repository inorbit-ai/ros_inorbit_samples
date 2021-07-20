cmake_minimum_required(VERSION 3.0.2)
project(inorbit_republisher)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslib
  std_msgs
)

catkin_package()

catkin_install_python(
  PROGRAMS
  scripts/republisher.py
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})