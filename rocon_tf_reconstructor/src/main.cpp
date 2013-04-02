#include <ros/ros.h>
#include <rocon_tf_reconstructor/rocon_tf_reconstructor.h>

int main(int argc, char ** argv)
{
  ros::init(argc,argv, "tf_reconstructor");
  ros::NodeHandle n;
  
  rocon::RoconTFReconstructor tr(n);

  ROS_INFO("Initialized");
  tr.spin();
  ROS_INFO("Bye Bye");
  return 0;
}
