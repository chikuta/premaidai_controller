
#ifndef PREMAIDAI_ROBOT_HW
#define PREMAIDAI_ROBOT_HW

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include "premaidai_ros_bridge/common.hh"
#include "premaidai_ros_bridge/ros_bridge.hh"

namespace premaidai_ros_bridge
{

    class RobotHW : public hardware_interface::RobotHW
    {
    public:
        RobotHW();
        virtual ~RobotHW();
        ros::Time getTime() const;
        ros::Duration getPeriod() const;
        void read();
        void write();
        int open();
        void close();

    private:
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::PositionJointInterface joint_position_interface_;
        DoubleArray commands_;
        DoubleArray positions_;
        DoubleArray velocities_;
        DoubleArray efforts_;
        ROSBridge::Ptr bridge_;
        sensor_msgs::JointState joint_state_;
    };

};

#endif // PREMAIDAI_ROBOT_HW