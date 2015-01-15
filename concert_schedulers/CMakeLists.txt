##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(concert_schedulers)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED)

catkin_package()
catkin_python_setup()

##############################################################################
# Tests
##############################################################################

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()

##############################################################################
# Installs
##############################################################################

install(
    PROGRAMS 
        scripts/compatibility_tree_scheduler.py
        scripts/concert_scheduler_requests
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
