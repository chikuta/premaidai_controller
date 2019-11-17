#include "premaidai_ros_bridge/serial_port.hh"

#include <boost/bind.hpp>
#include <ros/ros.h>

using namespace premaidai_ros_bridge;
using namespace boost::asio;

SerialPort::SerialPort(
        const std::string& port_name,
        unsigned int baud_rate,
        unsigned int byte_size,
        unsigned int stop_bits
    )
{
    io_service_ = IOServicePtr(new io_service());
    serial_port_ = SerialPortPtr(new serial_port(*io_service_, port_name));
    serial_port_->set_option(serial_port_base::baud_rate(baud_rate));
    serial_port_->set_option(serial_port_base::character_size(byte_size));
    serial_port_->set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
    serial_port_->set_option(serial_port_base::parity(serial_port_base::parity::none));
    serial_port_->set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
}

// virtual
SerialPort::~SerialPort()
{
    stop();
}


bool
SerialPort::start()
{
    if (!serial_port_ || !io_service_)
    {
        // output error message
        return false;
    }

    // start serial port
    asyncReadSome();

    // create async thread
    thread_ = ThreadPtr(new boost::thread(boost::bind(&io_service::run, io_service_)));

    return true;
}


bool
SerialPort::stop()
{
    // close serial port
    if (serial_port_)
    {
        serial_port_->cancel();
        serial_port_->close();
        serial_port_.reset();
    }

    // stop io service
    if (io_service_)
    {
        io_service_->stop();
        io_service_.reset();
    }

    // stop thread
    if (thread_)
    {
        thread_->join();
        thread_.reset();
    }

    return true;
}


void
SerialPort::write(const FrameType& data)
{
    serial_port_->async_write_some(
        boost::asio::buffer(data, data.size()),
        boost::bind(&SerialPort::writeHandler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred)
    );
}


FrameType
SerialPort::read(unsigned int data_size, bool erase_buffer)
{
    boost::mutex::scoped_lock(mutex_);

    FrameType ret;
    if (recv_data_.size() >= data_size)
    {
        ret.assign(recv_data_.begin(), recv_data_.begin() + data_size);
        if (erase_buffer)
        {
            recv_data_.erase(recv_data_.begin(), recv_data_.begin() + data_size);
        }
    }
    else
    {
        // error message
        ROS_DEBUG("SerialPort read error. SerialPort do not receive enough message. curr[%d] < except[%d]", (int)recv_data_.size(), (int)data_size);
    }

    return ret;
}

unsigned int
SerialPort::getReceiveDataSize() const
{
    return recv_data_.size();
}


void
SerialPort::asyncReadSome()
{
    boost::asio::streambuf::mutable_buffers_type mutable_buffer = recv_buffer_.prepare(1024);
    serial_port_->async_read_some(
        boost::asio::buffer(mutable_buffer),
        boost::bind(&SerialPort::readHandler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred)
    );
}


void
SerialPort::readHandler(const boost::system::error_code& ec, std::size_t bytes_transferred)
{
    boost::mutex::scoped_lock(mutex_);

    if(ec == boost::asio::error::operation_aborted)
    {
        // output error message
        ROS_WARN_STREAM("SerialPort read handler error. " << ec);
    }
    else
    {
        if (bytes_transferred > 0)
        {
            const uint8_t* buffer_ptr = boost::asio::buffer_cast<const uint8_t*>(recv_buffer_.data());
            recv_data_.insert(recv_data_.end(), buffer_ptr, buffer_ptr + bytes_transferred);
            recv_buffer_.consume(bytes_transferred);
        }
    }

    // set read event handler
    asyncReadSome();
}


void
SerialPort::writeHandler(const boost::system::error_code& ec, std::size_t bytes_transferred)
{
    if(ec == boost::asio::error::operation_aborted)
    {
        // output error message
        ROS_WARN_STREAM("SerialPort write handler error. " << ec);
    }
}
