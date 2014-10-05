cmake_minimum_required(VERSION 2.8.3)
project(concert_software_manager)

find_package(catkin REQUIRED COMPONENTS
  concert_msgs
  rospy
)

catkin_python_setup()
catkin_package()
