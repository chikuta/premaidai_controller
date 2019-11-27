#include "premaidai_ros_bridge/robot.hh"
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

using namespace premaidai_ros_bridge;

int
main(int argc, char* argv[])
{
    ros::init(argc, argv, "premaidai_driver");
    ros::NodeHandle nh("~");

    RobotHW robot = RobotHW();
    controller_manager::ControllerManager manager(&robot, nh);

    ros::Rate rate(1.0 / robot.getPeriod().toSec());
    ros::AsyncSpinner spinner(1);
    spinner.start();
    robot.open();

    while (ros::ok())
    {
        robot.read();
        manager.update(robot.getTime(), robot.getPeriod());
        robot.write();
        rate.sleep();
    }

    robot.close();
    return 0;
}