#include "premaidai_ros_bridge/ros_bridge.hh"
#include <ros/ros.h>

using namespace premaidai_ros_bridge;

int
main(int argc, char* argv[])
{
    ros::init(argc, argv, "premaidai_ros_bridge_node");
    ros::NodeHandle hn("~");

    ROSBridge::Ptr bridge = ROSBridge::Ptr(new ROSBridge());
    bridge->start();
    ros::spin();
}