#include "premaidai_controller/ros_bridge.hh"

using namespace premaidai_controller;

ROSBridge::ROSBridge()
    : nh_("~")
    , thread_()
    , is_shutdown_(false)
{
    // get baudrate
    int baudrate = 115200;
    if (!nh_.getParam("baudrate", baudrate))
    {
        ROS_INFO("baudrate set to default value [%u]", baudrate);
    }

    // get port name
    std::string port_name = "/dev/rfcomm0";
    if (!nh_.getParam("serial_port", port_name))
    {
        ROS_INFO("connection port set to default value [%s]", port_name.c_str());
    }

    // create serial port
    serial_port_ = SerialPortPtr(new SerialPort(
        port_name,
        baudrate,
        8,          // byte size
        1           // stop bits
    ));

    // get joint config from config file
    XmlRpc::XmlRpcValue list;
    if (nh_.getParam("joint_configs", list))
    {
        for (unsigned int idx = 0; idx < list.size(); ++idx)
        {
            JointConfig config;
            config.id = int(list[idx]["id"]);
            config.name = std::string(list[idx]["name"]);
            config.direction = int(list[idx]["direction"]);
            config.offset = int(list[idx]["offset"]);
            config_id_map_[config.id] = config;
            config_name_map_[config.name] = config;
            joint_names_.push_back(config.name);
        }
    }
    else
    {
        ROS_ERROR("parameter [joint_configs] is not found");
    }

    // initialize joint_state_
    joint_state_.name = joint_names_;
    joint_state_.position = DoubleArray(joint_names_.size(), 0);
    joint_state_.velocity = DoubleArray(joint_names_.size(), 0);
    joint_state_.effort = DoubleArray(joint_names_.size(), 0);

    // create topic
    joint_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);
    joint_sub_ = nh_.subscribe("joint_command", 10, &ROSBridge::jointStateCb, this);

    // create service
    request_motion_ = nh_.advertiseService("request_preset_motion", &ROSBridge::requestMotionCb, this);
    request_servo_off_ = nh_.advertiseService("request_servo_off", &ROSBridge::requestServoOffCb, this);
}


// virtual
ROSBridge::~ROSBridge()
{
    stop();
}


void
ROSBridge::start()
{
    serial_port_->start();
    thread_ = ThreadPtr(new boost::thread(boost::bind(&ROSBridge::run, this)));
}


void
ROSBridge::stop()
{
    serial_port_->stop();

    // end thread
    is_shutdown_ = true;
    thread_->join();
}


void
ROSBridge::requestServoOffMode()
{
    ServoOffRequestCommand cmd;
    optional_cmds_.push_back(cmd);
}


void
ROSBridge::executeMotion(const MotionType& command)
{
    MotionRequestCommand cmd(command);
    serial_port_->write(cmd.getPacket());
    optional_cmds_.push_back(cmd);
}


void
ROSBridge::sendServoPositionRequest(const DoubleArray& angles)
{
    if (angles.size() != joint_names_.size())
    {
        ROS_WARN("Invalid joint size. except=[%ld] actual=[%ld]", joint_names_.size(), angles.size());
        return;
    }

    ServoControlDataArray data_array;
    for (unsigned int idx = 0; idx < angles.size(); ++idx)
    {
        double angle = angles[idx];
        JointConfig config = config_name_map_[joint_names_[idx]];

        double rate = (angle * 180.0 / M_PI * config.direction + 135.0) / 270.0;
        uint16_t position = std::floor(rate * (EncoderMaxPosition - EncoderMinPosition) + EncoderMinPosition);

        ServoControlData data;
        data.id = config.id;
        data.position = position;
        data_array.push_back(data);
    }

    ServoControlCommand cmd(data_array);
    control_cmds_.push_back(cmd);
}


StringArray
ROSBridge::getJointNames() const
{
    return joint_names_;
}


sensor_msgs::JointState
ROSBridge::getJointState() const
{
    return joint_state_;
}


void
ROSBridge::run()
{
    // requesetの最大rateは20Hz
    ros::Rate rate(20);

    // create status requeset command for servo positions
    std::array<SendCommand, 2> status_cmds{
        ServoStatusRequestCommand(1, 16),
        ServoStatusRequestCommand(17, 17),
    };

    unsigned long count = 0;
    while (!is_shutdown_)
    {
        unsigned int idx = count % 4;
        if (idx == 0 || idx == 1)
        {
            serial_port_->write(status_cmds[idx].getPacket());
        }
        else if (idx == 2)
        {
            if (!control_cmds_.empty())
            {
                // send latest data
                serial_port_->write(control_cmds_.back().getPacket());
                control_cmds_.clear();
            }
        }
        else if (idx == 3)
        {
            if (!optional_cmds_.empty())
            {
                serial_port_->write(optional_cmds_.front().getPacket());
                optional_cmds_.erase(optional_cmds_.begin());
            }
            else if (!control_cmds_.empty())
            {
                // send latest data
                serial_port_->write(control_cmds_.back().getPacket());
                control_cmds_.clear();
            }
        }

        // process response
        while(processResponse())
        {
            ROS_DEBUG("Process response. Remaining data size = [%d]", (int)serial_port_->getReceiveDataSize());
        }

        // publish joint states
        updateJointStates();
        publishJointStates();

        ++count;
        rate.sleep();
    }
}


bool
ROSBridge::processResponse()
{
    // get data type
    FrameData length = serial_port_->read(1, false);
    if (length.size() == 0)
    {
        ROS_DEBUG("No data on receive queue.");
        return false;
    }

    // get whole data
    FrameData data = serial_port_->read(length[0], true);
    if (data.size() != length[0])
    {
        ROS_DEBUG("Drop data. data length = [%d] / curr length = [%d]", length[0], (int)data.size());
        return false;
    }

    // check command and validate response
    const uint8_t type = data[1];
    if (type == static_cast<uint8_t>(CommandType::GET_PARAM))
    {
        ServoStatusResponseCommand cmd(data);
        if (cmd.accepted())
        {
            servo_status_.push_back(cmd.getServoStatus());
        }
    }
    else if (type == static_cast<uint8_t>(CommandType::STOP))
    {
        ROS_INFO("STOP command is not implemented");
    }
    else if (type == static_cast<uint8_t>(CommandType::SET_SERVO_POS))
    {
        ServoControlResponseCommand cmd(data);
        if (cmd.accepted())
        {
            ROS_DEBUG("ServoStatusRequest command accepted.");
        }
        else
        {
            ROS_WARN("ServoStatusRequeset command not accepted.");
        }
    }
    else if (type == static_cast<uint8_t>(CommandType::SET_SERVO_PARAM))
    {
        ROS_INFO("SET_SERVO_PARAM command is not implemented");
    }
    else if (type == static_cast<uint8_t>(CommandType::SET_LED_CMD))
    {
        ROS_INFO("SET_LED_CMD command is not implemented");
    }
    else if (type == static_cast<uint8_t>(CommandType::PLAY_MOTION))
    {
        MotionResponseCommand cmd(data);
        if (cmd.accepted())
        {
            ROS_INFO("MotionRequest is accepted.");
        }
    }
    else
    {
        ROS_WARN("Unknown response command[%d] received.", type);
    }

    return true;
}


void
ROSBridge::updateJointStates()
{
    for (auto servo_status : servo_status_)
    {
        for (auto state : servo_status)
        {
            if (config_id_map_.find(state.id) != config_id_map_.end())
            {
                JointConfig config = config_id_map_[state.id];
                double angle = float(state.actual_position - EncoderMinPosition) / (EncoderMaxPosition - EncoderMinPosition) * 270.0 - 135.0;

                // find index of joint_name
                StringArray::iterator it = std::find(joint_state_.name.begin(), joint_state_.name.end(), config.name);
                size_t idx = std::distance(joint_state_.name.begin(), it);
                if (idx == joint_state_.name.size())
                {
                    ROS_WARN("Unregistered joint_name [%s]", config.name.c_str());
                }
                else
                {
                    joint_state_.position[idx] = (angle + config.offset) / 180.0 * M_PI * config.direction;
                }
            }
        }
    }

    // update timestamp if renew
    if (servo_status_.size())
    {
        ++joint_state_.header.seq;
        joint_state_.header.stamp = ros::Time::now();
        servo_status_.clear();
    }
}


void
ROSBridge::jointStateCb(const sensor_msgs::JointState& joint_state)
{
    // sort names
    DoubleArray data;
    StringArray names = joint_state.name;
    for (auto name : joint_names_)
    {
        StringArray::iterator it = std::find(names.begin(), names.end(), name);
        size_t idx = std::distance(names.begin(), it);
        if (idx != names.size())
        {
            data.push_back(joint_state.position[idx]);
        }
    }
    sendServoPositionRequest(data);
}


void
ROSBridge::publishJointStates()
{
    joint_pub_.publish(joint_state_);
}


bool
ROSBridge::requestMotionCb(MotionRequest::Request& request, MotionRequest::Response& response)
{
    executeMotion(static_cast<MotionType>(request.motion_command));
    response.result = true;
    return true;
}


bool
ROSBridge::requestServoOffCb(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
    requestServoOffMode();
    response.success = true;
    return true;
}
