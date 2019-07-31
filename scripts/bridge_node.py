#!/usr/bin/env python
# -*- coding: utf-8 -*-

# import local package
from premaidai_ros_bridge.client import Controller

# import ros related packages
import rospy
from sensor_msgs.msg import JointState


def main():
    rospy.init_node('joint_state_publisher')
    joint_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    port_name = rospy.get_param("~serial_port", "/dev/rfcomm0")
    file_name = rospy.get_param("~config_file", "NONE")

    controller = Controller('/dev/rfcomm0', file_name)
    controller.start()


    # create servo off command
    if rospy.get_param('~servo_off_mode', False):
        controller.power_off()

    while not rospy.is_shutdown():
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_angle_dict = controller.get_joint_status()

        for name, angle in joint_angle_dict.items():
            joint_state.name.append(name)
            joint_state.position.append(angle)

        joint_pub.publish(joint_state)

    controller.shutdown()


if __name__ == '__main__':
    main()
