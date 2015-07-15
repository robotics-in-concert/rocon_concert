##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(concert_conductor)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED dynamic_reconfigure)

catkin_python_setup()
generate_dynamic_reconfigure_options(cfg/params.cfg)
catkin_package()

##############################################################################
# Installs
##############################################################################

install(
    PROGRAMS 
        scripts/conductor.py
        scripts/concert_conductor_graph
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
