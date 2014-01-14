##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(concert_service_utilities)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED COMPONENTS rostest)
catkin_package()

catkin_python_setup()

##############################################################################
# Tests
##############################################################################

if(CATKIN_ENABLE_TESTING)
   add_rostest(test/concert_info.rostest)
endif()

##############################################################################
# Installs
##############################################################################

#install(
#    PROGRAMS 
#        scripts/rocon_interactions
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#    )

#install(DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
