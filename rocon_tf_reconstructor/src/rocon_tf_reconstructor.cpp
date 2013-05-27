#include <rocon_tf_reconstructor/rocon_tf_reconstructor.h>

namespace rocon {

  RoconTFReconstructor::RoconTFReconstructor()
  {
    this->nh = ros::NodeHandle("");
    init();
  }
  RoconTFReconstructor::RoconTFReconstructor(ros::NodeHandle& n)
  {
    this->nh = n;
    init();
  }


  RoconTFReconstructor::~RoconTFReconstructor()
  {
  }

  void RoconTFReconstructor::init()
  {
    getParams();
    setSubscriber();
  }

  void RoconTFReconstructor::getParams()
  {
    this->nh.param<std::string>("list_concert_client",this->sub_client_list_topic,"/concert/list_concert_clients");
    this->nh.param<std::string>("robotpose_topic",this->sub_robotpose_topic,"robot_pose");
  }


  void RoconTFReconstructor::setSubscriber()
  {
    this->sub_clientlist = this->nh.subscribe(this->sub_client_list_topic,10,&RoconTFReconstructor::processClientLists,this);
  }

  void RoconTFReconstructor::processClientLists(const concert_msgs::ConcertClients::ConstPtr msg)
  {
    int i;
    std::string name;
    std::vector<std::string> client_names;
    // TODO : Optimize synchronization using sorting algorithm

    // register newly joined clients
    for(i = 0; i <msg->clients.size(); i ++) {
      name = msg->clients[i].name;
      client_names.push_back(name);

      // if it doesn't exist create subscriber
      if(this->sub_clients_pose.find(name) == this->sub_clients_pose.end())
      {
        std::string topic_name = "/" + name + "/" + this->sub_robotpose_topic;  
        ROS_INFO_STREAM("Create Subscriber for : " << name << "\tTopic : " << topic_name);
        this->sub_clients_pose[name] = new RoconPoseClient(this->nh,name,this->sub_robotpose_topic);
      }
    }

    // remove newly left clients
    std::map<std::string,RoconPoseClient*>::iterator iter;
    for(iter = this->sub_clients_pose.begin(); iter != this->sub_clients_pose.end(); ++iter)
    {
      std::string key = iter->first;

      // it does not exist in msg client name array. so this client has left concert.
      if(std::find(client_names.begin(),client_names.end(),key) == client_names.end())
      {
        ROS_INFO_STREAM("Remove subscriber of : " << name);
        this->sub_clients_pose.erase(key);
      }
    }
  }

  void RoconTFReconstructor::publishClientTF(std::string client_name,geometry_msgs::PoseStamped pose_stamped)
  {
    geometry_msgs::TransformStamped odom_tf;

    odom_tf.header = pose_stamped.header;
    odom_tf.header.stamp = ros::Time::now();
    odom_tf.child_frame_id = client_name;
    odom_tf.transform.translation.x = pose_stamped.pose.position.x;
    odom_tf.transform.translation.y = pose_stamped.pose.position.y;
    odom_tf.transform.translation.z = pose_stamped.pose.position.z;
    odom_tf.transform.rotation = pose_stamped.pose.orientation;

    this->tf_broadcaster.sendTransform(odom_tf);
  }

  void RoconTFReconstructor::spin()
  {
    ROS_INFO("In Spin!");
    RoconPoseClient* rpc;
    ros::Rate spin_rate(1000);

    while(ros::ok())
    {
      ros::spinOnce();
      std::map<std::string,RoconPoseClient*>::iterator iter;
      for(iter = this->sub_clients_pose.begin(); iter != this->sub_clients_pose.end(); ++iter)
      {
        rpc = iter->second;
        if(rpc->isInitialized()) {
          std::string client_name = rpc->getClientName();
          geometry_msgs::PoseStamped client_pose = rpc->getPoseStamped();

          publishClientTF(client_name,client_pose);
        }
      }
      spin_rate.sleep();
    }
  }
}
