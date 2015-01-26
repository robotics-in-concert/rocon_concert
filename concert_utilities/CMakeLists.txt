##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(concert_utilities)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED)
catkin_package(
    CATKIN_DEPENDS rospy 
)

catkin_python_setup()

##############################################################################
# Installs
##############################################################################

install(
    PROGRAMS 
        scripts/conductor_graph_to_string.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

#install(DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


