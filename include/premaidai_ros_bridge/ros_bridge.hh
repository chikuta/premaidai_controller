#ifndef PREMAIDAI_ROS_BRIDGE
#define PREMAIDAI_ROS_BRIDGE

#include <memory>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <boost/thread.hpp>
#include <sensor_msgs/JointState.h>
#include "premaidai_ros_bridge/command.hh"
#include "premaidai_ros_bridge/common.hh"
#include "premaidai_ros_bridge/serial_port.hh"
#include "premaidai_ros_bridge/MotionRequest.h"

namespace premaidai_ros_bridge
{
    class ROSBridge
    {
    public:
        typedef std::shared_ptr<ROSBridge> Ptr;

        ROSBridge();
        virtual ~ROSBridge();
        void start();
        void stop();
        void requestServoOffMode();
        void executeMotion(const MotionType& command);
        void sendServoPositionRequest(const DoubleArray& angles);
        StringArray getJointNames() const;
        sensor_msgs::JointState getJointState() const;

    protected:
        void run();
        bool processResponse();
        void publishJointStates();
        void updateJointStates();
        void jointStateCb(const sensor_msgs::JointState& joint_state);

        /**
         * @brief callback function for "request_preset_motion" service.
         */
        bool requestMotionCb(MotionRequest::Request& request, MotionRequest::Response& response);

        /**
         * @brief callback function for "request_servo_off" service.
         */
        bool requestServoOffCb(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);

    private:
        // typedefs
        typedef std::shared_ptr<boost::thread> ThreadPtr;
        typedef std::shared_ptr<SerialPort> SerialPortPtr;
        typedef std::vector<ServoStatusArray> ServoStatus2DimArray;
        typedef std::vector<SendCommand> SendCommandArray;

        ros::NodeHandle nh_;                    /**< Node handler */
        ThreadPtr thread_;                      /**< thread pointer */
        SerialPortPtr serial_port_;             /**< serial port async connection */
        ServoStatus2DimArray servo_status_;     /**< raw servo data from premaidai */
        JointIdConfigMap config_id_map_;        /**< servo config id map from rosparam */
        JointNameConfigMap config_name_map_;    /**< servo config name map from rosparam */
        StringArray joint_names_;               /**< joint names */
        SendCommandArray control_cmds_;         /**< send command array */
        SendCommandArray optional_cmds_;        /**< send command array for optional */
        sensor_msgs::JointState joint_state_;   /**< current joint state */
        bool is_shutdown_;                      /**< loop keeper */
        ros::Publisher joint_pub_;              /**< topic publisher for joint state */
        ros::Subscriber joint_sub_;             /**< joint command subscriber */
        ros::ServiceServer request_motion_;     /**< service server for preset motion request */
        ros::ServiceServer request_servo_off_;  /**< service server for servo off */
    };
};

#endif // PREMAIDAI_ROS_BRIDGE