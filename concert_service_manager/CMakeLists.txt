##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(concert_service_manager)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED)

catkin_python_setup()
catkin_package()

##############################################################################
# Unit Tests
##############################################################################

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()

##############################################################################
# Installs
##############################################################################

install(
    PROGRAMS 
        scripts/service_manager
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
