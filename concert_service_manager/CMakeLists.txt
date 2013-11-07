cmake_minimum_required(VERSION 2.8.3)
project(concert_service_manager)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS rospy 
)

install(PROGRAMS
  scripts/service_manager
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
