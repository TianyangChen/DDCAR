#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block uCanDoIt/Subscribe
extern SimulinkSubscriber<geometry_msgs::Twist, SL_Bus_uCanDoIt_geometry_msgs_Twist> Sub_uCanDoIt_1;

// For Block uCanDoIt/Publish
extern SimulinkPublisher<std_msgs::Bool, SL_Bus_uCanDoIt_std_msgs_Bool> Pub_uCanDoIt_79;

void slros_node_init(int argc, char** argv);

#endif
