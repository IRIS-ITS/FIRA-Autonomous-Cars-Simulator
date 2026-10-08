cmake_minimum_required(VERSION 3.0.2)
project(diffdrive2gazebo)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
)

add_message_files(
  FILES
  WheelVel.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime rospy std_msgs
)

catkin_install_python(PROGRAMS scripts/diffdrive2gazebo.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
