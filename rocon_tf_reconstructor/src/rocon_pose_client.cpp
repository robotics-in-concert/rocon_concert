#include<rocon_tf_reconstructor/rocon_pose_client.h>

namespace rocon {

  RoconPoseClient::RoconPoseClient()
  {
    this->initialized = false;
  }

  RoconPoseClient::RoconPoseClient(ros::NodeHandle& nh,std::string client_name,std::string pose_topic)
  {
    ros::NodeHandle n;
    std::string topic_name = "/" + client_name + "/" + pose_topic;  

    this->initialized = false;
    this->client_name = client_name;
    this->pose_topic = pose_topic;
    this->sub = n.subscribe(topic_name,10,&RoconPoseClient::processPose,this);
  }

  RoconPoseClient::~RoconPoseClient()
  {
    this->sub.shutdown();
  }


  void RoconPoseClient::processPose(const geometry_msgs::PoseStamped::ConstPtr msg)
  {
    this->initialized = true;
    this->pose_stamped = *msg;
  }

  geometry_msgs::PoseStamped RoconPoseClient::getPoseStamped()
  {
    return this->pose_stamped;
  }

  std::string RoconPoseClient::getClientName()
  {
    return this->client_name;
  }

  bool RoconPoseClient::isInitialized()
  {
    return this->initialized;
  }
}
