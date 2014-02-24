##############################################################################
# RosTests - only run when CATKIN_ENABLE_TESTING is true
##############################################################################

# Unit tests using nose, but needing a running ROS core.
find_package(catkin REQUIRED COMPONENTS rostest)

#add_rostest(py_example_requester.test)
#add_rostest(py_example_scheduler.test)
#add_rostest(py_timeout.test)
