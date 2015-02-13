cmake_minimum_required(VERSION 2.8.3)
project(concert_software_farmer)

find_package(catkin REQUIRED)

catkin_python_setup()
catkin_package()

install(
  PROGRAMS
    scripts/software_farmer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
