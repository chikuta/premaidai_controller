#include <premaidai_ros_bridge/robot.hh>

using namespace premaidai_ros_bridge;


RobotHW::RobotHW()
    : hardware_interface::RobotHW()
{}


// virtual
RobotHW::~RobotHW()
{
    bridge_->stop();
}


ros::Time
RobotHW::getTime() const
{
    return ros::Time::now();
}


ros::Duration
RobotHW::getPeriod() const
{
    return ros::Duration(0.05);
}


void
RobotHW::read()
{
    joint_state_ = bridge_->getJointState();

    // check data size
    if (joint_state_.position.size() == positions_.size())
    {
        positions_ = joint_state_.position;
    }
    else
    {
        ROS_WARN("Invalid joint position size. expect=[%lu] actual=[%lu].", positions_.size(), joint_state_.position.size());
    }

    if (joint_state_.velocity.size() == velocities_.size())
    {
        velocities_ = joint_state_.velocity;
    }
    else
    {
        ROS_WARN("Invalid joint velocity size. expect=[%lu] actual=[%lu].", velocities_.size(), joint_state_.velocity.size());
    }

    if (joint_state_.effort.size() == efforts_.size())
    {
        efforts_ = joint_state_.effort;
    }
    else
    {
        ROS_WARN("Invalid joint effort size. expect=[%lu] actual=[%lu].", efforts_.size(), joint_state_.effort.size());
    }
}


void
RobotHW::write()
{
    bridge_->sendServoPositionRequest(commands_);
}


int
RobotHW::open()
{
    // create bridge
    bridge_ = ROSBridge::Ptr(new ROSBridge());
    const StringArray joint_names = bridge_->getJointNames();

    // setup data size
    positions_ = DoubleArray(joint_names.size(), 0);
    velocities_ = DoubleArray(joint_names.size(), 0);
    efforts_ = DoubleArray(joint_names.size(), 0);
    commands_ = DoubleArray(joint_names.size(), 0);

    for (unsigned int idx = 0; idx < joint_names.size(); ++idx)
    {
        // register joint state handles
        hardware_interface::JointStateHandle joint_state_handle(joint_names[idx], &positions_[idx], &velocities_[idx], &efforts_[idx]);
        joint_state_interface_.registerHandle(joint_state_handle);

        // register joint position handles
        hardware_interface::JointHandle pos_handle(joint_state_interface_.getHandle(joint_names[idx]), &commands_[idx]);
        joint_position_interface_.registerHandle(pos_handle);
    }

    // register interfaces
    registerInterface(&joint_state_interface_);
    registerInterface(&joint_position_interface_);

    // start bridge
    bridge_->start();
}


void
RobotHW::close()
{
    bridge_->stop();
}
