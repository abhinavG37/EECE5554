//
// Created by abhinav137 on 17/07/21.
//
#include <ros/ros.h>
#include <std_msgs/String.h>

void chatterCallback(const std_msgs::String::ConstPtr & msg){
  ROS_INFO("Message Heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv){
  ros::init(argc, argv, "PA1Listener");
  ros::NodeHandle node_listener;
  ros::Subscriber chatter_sub = node_listener.subscribe("PA1Talker", 1000, chatterCallback);
  ros::spin();
  return 0;
}