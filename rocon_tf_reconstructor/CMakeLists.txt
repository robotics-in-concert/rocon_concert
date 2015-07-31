cmake_minimum_required(VERSION 2.8.3)
project(rocon_tf_reconstructor)

find_package(catkin REQUIRED COMPONENTS concert_msgs geometry_msgs roscpp tf)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  LIBRARIES ${PROJECT_NAME}_lib
  INCLUDE_DIRS include
  CATKIN_DEPENDS concert_msgs geometry_msgs roscpp tf
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}_lib
            src/rocon_tf_reconstructor.cpp
            src/rocon_pose_client.cpp
            src/utils.cpp
)
set_target_properties(${PROJECT_NAME}_lib PROPERTIES OUTPUT_NAME rocon_tf_reconstructor)
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_lib concert_msgs_gencpp)

## Declare a cpp executable
add_executable(rocon_tf_reconstructor src/main.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(
  rocon_tf_reconstructor
  ${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
)

install(TARGETS rocon_tf_reconstructor_lib 
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Mark executables and/or libraries for installation
install(TARGETS rocon_tf_reconstructor
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

