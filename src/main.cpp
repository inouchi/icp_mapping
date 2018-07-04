#include <ros/ros.h>
#include <iostream>
#include "ICPMapper.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "icp_mapper");
  ROS_INFO("Starting icp_mapper-node with name %s", ros::this_node::getName().c_str());
  ros::NodeHandle nodeHandle;
  ros::NodeHandle localNodeHandle("~");

  // Create a ICPMapping instance
  ICPMapper icpMapper(&nodeHandle, &localNodeHandle);
  
  while (ros::ok())
  {
    ros::spinOnce();
  }

  return 0;
}
