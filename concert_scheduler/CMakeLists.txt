##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(concert_scheduler)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED)

catkin_package()
catkin_python_setup()

##############################################################################
# Installs
##############################################################################

install(
    PROGRAMS 
        scripts/compatibility_tree_scheduler
        scripts/scheduler
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
