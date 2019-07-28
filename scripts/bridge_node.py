#!/usr/bin/env python
# -*- coding: utf-8 -*-

# import default packages
import math
from collections import namedtuple

# import local package
from premaidai_ros_bridge.client import Connector, ServoOffCommand, ServoStatusCommand

# import ros related packages
import rospy
from sensor_msgs.msg import JointState


def main():
    rospy.init_node('joint_state_publisher')
    joint_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    port_name = rospy.get_param("~serial_port", "/dev/rfcomm0")
    joint_configs = rospy.get_param("~joint_configs", [])

    con = Connector(port_name, 115200)
    con.start()

    JointConfig = namedtuple("JointConfig", ["id", "name", "direction", "offset"])
    joint_config_dict = {}
    for config in joint_configs:
        joint_config = JointConfig(**config)
        joint_config_dict[joint_config.id] = joint_config

    # create servo off command
    if rospy.get_param('~servo_off_mode', False):
        servo_off_command = ServoOffCommand()
        con.send_command(servo_off_command)
        res = con.get_response()

    while not rospy.is_shutdown():
        servo_data = []

            # ServoStatusCommand(2, 15),
            # ServoStatusCommand(17, 10)
        servo_commands = [
            ServoStatusCommand(1, 16),
            ServoStatusCommand(17, 17)
        ]

        # send request
        for cmd in servo_commands:
            con.send_command(cmd)

        # get all servo data
        servo_data_list = con.get_response().get_response()
        servo_data_list.extend(con.get_response().get_response())

        # make joint state msg
        joints = JointState()
        joints.header.stamp = rospy.Time.now()

        for servo_data in servo_data_list:
            if servo_data.id in joint_config_dict:
                angle = (servo_data.actual_position - 3500.0) / (11500 - 3500) * 270.0 - 135.0
                joint_config = joint_config_dict[servo_data.id]
                joints.position.append((angle + joint_config.offset) / 180.0 * math.pi * joint_config.direction)
                joints.name.append(joint_config.name)

        joint_pub.publish(joints)

    con.shutdown()

if __name__ == '__main__':
    main()
