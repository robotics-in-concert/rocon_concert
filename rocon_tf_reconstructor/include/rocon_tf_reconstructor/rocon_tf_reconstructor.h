/*
# Software License Agreement (BSD License)
#
# Copyright (c) 2012 Yujin Robot, Jihoon Lee
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#    * Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above
#        copyright notice, this list of conditions and the following
#        disclaimer in the documentation and/or other materials provided
#        with the distribution.
#    * Neither the name of Yujin Robot nor the names of its
#        contributors may be used to endorse or promote products derived
#        from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef ROCON_TF_RECONSTRUCTOR_ROCON_TF_RECONSTRUCTOR_H_
#define ROCON_TF_RECONSTRUCTOR_ROCON_TF_RECONSTRUCTOR_H_

#include<ros/ros.h>
#include<tf/tf.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_datatypes.h>
#include<concert_msgs/ConcertClients.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/TransformStamped.h>
#include "rocon_pose_client.h"

namespace rocon {

  class RoconTFReconstructor {
    public:
      RoconTFReconstructor();
      RoconTFReconstructor(ros::NodeHandle& n);
      ~RoconTFReconstructor();
      void spin();

    protected:
      void setSubscriber();
      void getParams();
      void init();

      void processClientLists(const concert_msgs::ConcertClients::ConstPtr msg);
      void publishClientTF(std::string client_name,geometry_msgs::PoseStamped pose_stamped);

    private:
      ros::NodeHandle nh;

      tf::TransformBroadcaster tf_broadcaster;
      ros::Subscriber sub_clientlist;

      std::vector<std::string> clients;
      std::map<std::string,RoconPoseClient*> sub_clients_pose;

      std::string sub_client_list_topic;
      std::string sub_robotpose_topic;

  };
}

#endif
