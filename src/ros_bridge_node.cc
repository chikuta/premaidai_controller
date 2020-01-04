#include "premaidai_controller/ros_bridge.hh"
#include <ros/ros.h>

using namespace premaidai_controller;

int
main(int argc, char* argv[])
{
    ros::init(argc, argv, "premaidai_controller_node");
    ros::NodeHandle hn("~");

    ROSBridge::Ptr bridge = ROSBridge::Ptr(new ROSBridge());
    bridge->start();
    ros::spin();
}