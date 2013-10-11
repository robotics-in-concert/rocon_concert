cmake_minimum_required(VERSION 2.8.3)
project(concert_servicemanager)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS 
)

install(PROGRAMS
  nodes/concert_servicemanager
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
