cmake_minimum_required(VERSION 3.0.2)
project(proj_bixos)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  ros_nodes/vision_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

