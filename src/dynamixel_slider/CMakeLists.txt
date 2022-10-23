cmake_minimum_required(VERSION 3.0.2)
project(dynamixel_slider)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui_py
)

catkin_package()

catkin_install_python(PROGRAMS slider_publisher
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
