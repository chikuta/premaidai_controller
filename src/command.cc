#include "premaidai_controller/command.hh"

#include <ros/ros.h>
#include <sstream>
#include <cmath>
#include <boost/format.hpp>

using namespace premaidai_controller;

// explicit
Command::Command(const CommandType& type)
    : data_()
    , type_(type)
{}


// virtual
Command::~Command()
{}


bool
Command::validatePacket()
{
    // check data size
    if (data_.size() < 2)
    {
        ROS_WARN_STREAM("Invalid data size [" << data_.size() << "].");
        return false;
    }

    if (data_.size() != data_[0])
    {
        ROS_WARN("Invalid data size. expected [%d], actual [%d]", data_[0], (int)data_.size());
        return false;
    }

    // create check byte
    uint8_t check_byte = 0x00;
    for (FrameData::iterator it = data_.begin(); it < data_.end(); ++it)
    {
        if (it == data_.begin())
        {
            check_byte = *it;
        }
        else if (it != data_.end() - 1)
        {
            check_byte ^= *it;
        }
    }

    if (check_byte != data_.back())
    {
        ROS_DEBUG("Parity error. expect [%d], actual [%d]", check_byte, data_.back());
        // return false;
    }
    return true;
}


void
Command::printPacket()
{
    FrameData packet = data();
    std::stringstream ss;
    for (uint8_t byte : packet)
    {
        ss << boost::format("%02x") % byte << " ";
    }
    ROS_INFO("%s", ss.str().c_str());
}


FrameData
Command::data() const
{
    return data_;
}


void
Command::data(const FrameData& data)
{
    data_ = data;
}


CommandType
Command::type() const
{
    return type_;
}


// explicit
SendCommand::SendCommand(const CommandType& type)
    : Command(type)
{}

// virtual
SendCommand::~SendCommand()
{}


FrameData
SendCommand::getPacket() const
{
    return data();
}


FrameData
SendCommand::finalizePacket(const FrameData& data) const
{
    // create header
    // command length | command type | parameter(0x00)
    FrameData ret = data;
    ret.insert(ret.begin(), static_cast<uint8_t>(type()));      // command type
    ret.insert(ret.begin() + 1, 0x00);                          // parameter (0x00)
    ret.insert(ret.begin(), ret.size() + 2);                    // command length. +2 means self + parity byte

    // add parity
    uint8_t parity = 0x00;
    for (FrameData::iterator it = ret.begin(); it < ret.end(); ++it)
    {
        if (it == ret.begin())
        {
            parity = *it;
        }
        else
        {
            parity ^= *it;
        }
    }
    ret.push_back(parity);

    return ret;
}


MotionRequestCommand::MotionRequestCommand(const MotionType& motion)
    : SendCommand(CommandType::PLAY_MOTION)
{
    FrameData packet;
    packet.push_back(static_cast<uint8_t>(motion));
    data(finalizePacket(packet));
}


// virtual
MotionRequestCommand::~MotionRequestCommand()
{}


ServoStatusRequestCommand::ServoStatusRequestCommand(unsigned int start_id, unsigned int servo_num)
    : SendCommand(CommandType::GET_PARAM)
{
    const unsigned int max_servo_num = 17;
    const unsigned int servo_packet_len = 14;
    const unsigned int min_servo_id = 1;
    const unsigned int max_servo_id = 27;

    // check arguments
    if (max_servo_num < start_id)
    {
        ROS_ERROR("Invalid servo id [%u]", start_id);
    }

    FrameData packet;
    packet.push_back(0x05);                              // get servo status
    packet.push_back(start_id);                          // servo start id
    packet.push_back(servo_num * servo_packet_len);      // dump data size
    data(finalizePacket(packet));
}


// virtual
ServoStatusRequestCommand::~ServoStatusRequestCommand()
{}


ServoOffRequestCommand::ServoOffRequestCommand()
    : SendCommand(CommandType::SET_SERVO_POS)
{
    unsigned int min_servo_id = 1;
    unsigned int max_servo_id = 30;

    FrameData packet;
    packet.push_back(0x80);
    for (unsigned int idx = min_servo_id; idx <= max_servo_id; ++idx)
    {
        packet.push_back(idx);
        packet.push_back(0x00);
        packet.push_back(0x00);
    }
    data(finalizePacket(packet));
}


// virtual
ServoOffRequestCommand::~ServoOffRequestCommand()
{}


ServoControlCommand::ServoControlCommand(const ServoControlDataArray& data_array)
    : SendCommand(CommandType::SET_SERVO_POS)
{
    FrameData packet;
    packet.push_back(0x80);
    for (auto data : data_array)
    {
        packet.push_back(data.id);
        packet.push_back(data.position & 0xFF);
        packet.push_back((data.position >> 0x08) & 0xFF);
    }
    data(finalizePacket(packet));
}


// virtual
ServoControlCommand::~ServoControlCommand()
{}


// explicit
ReceiveCommand::ReceiveCommand(const CommandType& type, const FrameData& packet)
    : Command(type)
    , is_accepted_(false)
{
    data(packet);

    // unpack data
    is_accepted_ = unpack();
}


// virtual
ReceiveCommand::~ReceiveCommand()
{}


bool
ReceiveCommand::accepted() const
{
    return is_accepted_;
}


void
ReceiveCommand::accepted(bool accept)
{
    is_accepted_ = accept;
}


// virtual
bool
ReceiveCommand::unpack()
{
    FrameData packet = data();
    if (packet[1] != static_cast<uint8_t>(type()))
    {
        ROS_WARN("Unexpected command type. except=[%d] actual=[%d]", static_cast<int>(type()), static_cast<int>(packet[0]));
        return false;
    }

    if (packet[2] == 0x80)
    {
        ROS_WARN("Command execution failed.");
        return false;
    }

    return validatePacket();
}


// explicit
MotionResponseCommand::MotionResponseCommand(const FrameData& data)
    : ReceiveCommand(CommandType::PLAY_MOTION, data)
{}


// virtual
MotionResponseCommand::~MotionResponseCommand()
{}


// explicit
ServoStatusResponseCommand::ServoStatusResponseCommand(const FrameData& data)
    : ReceiveCommand(CommandType::GET_PARAM, data)
{
    accepted(unpack());
}


// virtual
ServoStatusResponseCommand::~ServoStatusResponseCommand()
{}


ServoStatusArray
ServoStatusResponseCommand::getServoStatus() const
{
    return servo_status_;
}


// virtual
bool
ServoStatusResponseCommand::unpack()
{
    // check data size, parity
    if (!ReceiveCommand::unpack())
    {
        return false;
    }

    // get servo status
    FrameData packet = data();
    const unsigned int servo_info_size = 14;
    unsigned int servo_num = (packet.size() - 4) / servo_info_size;  // 4 => header x 3 byte + parity * 1 byte

    for (unsigned int idx = 0; idx < servo_num; ++idx)
    {
        ServoStatus status;
        unsigned int offset = servo_info_size * idx + 3;    // 2 = header x 2 byte
        FrameData servo_info(packet.begin() + offset, packet.begin() + offset + servo_info_size);
        status.id = servo_info[0];
        status.enable = (servo_info[1] == 0x80) ? true : false;
        status.actual_position = static_cast<uint16_t>(servo_info[3] << 0x08 | servo_info[2]);
        status.offset = static_cast<uint16_t>(servo_info[5] << 0x08 | servo_info[4]);
        status.command_position = static_cast<uint16_t>(servo_info[7] << 0x08 | servo_info[6]);
        status.gyro_direction = static_cast<uint16_t>(servo_info[9] << 0x08 | servo_info[8]);
        status.gyro_scale = static_cast<uint16_t>(servo_info[11] << 0x08 | servo_info[10]);
        status.speed = servo_info[12];
        status.stretch = servo_info[13];
        servo_status_.push_back(status);
    }

    return true;
}


// explicit 
ServoControlResponseCommand::ServoControlResponseCommand(const FrameData& data)
    : ReceiveCommand(CommandType::SET_SERVO_POS, data)
{}


// virtual
ServoControlResponseCommand::~ServoControlResponseCommand()
{}

