##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(concert_orchestra)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()
catkin_add_nosetests(test/test_compatibles.py)

##############################################################################
# Installs
##############################################################################

install(
    PROGRAMS 
        scripts/impl_to_dot
        scripts/orchestra.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
