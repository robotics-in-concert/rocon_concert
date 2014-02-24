##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(rocon_interactions)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

##############################################################################
# Unit Tests
##############################################################################

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests)
endif()

##############################################################################
# Installs
##############################################################################

install(
    PROGRAMS 
        scripts/rocon_interactions
        scripts/rocon_interactions_manager.py
        scripts/load_roles_and_apps
        scripts/attach_test_remocon
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
