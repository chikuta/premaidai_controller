#ifndef _SERIAL_PORT_HH_
#define _SERIAL_PORT_HH_

#include <memory>
#include <string>
#include <vector>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/thread.hpp>


namespace premaidai_controller
{
    typedef std::vector<uint8_t> FrameType;

    /**
     * @brief SerialPort class
     */
    class SerialPort
    {
    public:
        SerialPort(
            const std::string& port_name,
            unsigned int baud_rate,
            unsigned int byte_size,
            unsigned int stop_bits
        );

        /**
         * @brief Destroy the Serial Connection object
         */
        virtual ~SerialPort();

        bool start();
        bool stop();
        void write(const FrameType& data);
        FrameType read(unsigned int data_size, bool erase_buffer = true);
        unsigned int getReceiveDataSize() const;

    private:
        typedef std::shared_ptr<boost::asio::io_service> IOServicePtr;
        typedef std::shared_ptr<boost::asio::serial_port> SerialPortPtr;
        typedef std::shared_ptr<boost::thread> ThreadPtr;

        void asyncReadSome();
        void writeHandler(const boost::system::error_code& ec, std::size_t bytes_transferred);
        void readHandler(const boost::system::error_code& ec, std::size_t bytes_transferred);

        std::string port_;
        IOServicePtr io_service_;
        SerialPortPtr serial_port_;
        ThreadPtr thread_;
        FrameType recv_data_;
        boost::asio::streambuf recv_buffer_;
        boost::mutex mutex_;
    };
};

#endif // _SERIAL_PORT_HH_