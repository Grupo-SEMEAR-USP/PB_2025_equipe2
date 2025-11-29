cmake_minimum_required(VERSION 3.0.2)
project(pb_2025)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
)

catkin_package()

catkin_install_python(
    PROGRAMS
        scripts/hardware_interface.py
        scripts/vision_node.py
        scripts/navigation_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

