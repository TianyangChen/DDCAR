#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "uCanDoIt";

// For Block uCanDoIt/Subscribe
SimulinkSubscriber<geometry_msgs::Twist, SL_Bus_uCanDoIt_geometry_msgs_Twist> Sub_uCanDoIt_1;

// For Block uCanDoIt/Publish
SimulinkPublisher<std_msgs::Bool, SL_Bus_uCanDoIt_std_msgs_Bool> Pub_uCanDoIt_79;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

