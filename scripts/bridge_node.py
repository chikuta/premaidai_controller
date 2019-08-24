#!/usr/bin/env python
# -*- coding: utf-8 -*-

# import pthon packages
import math
import time
from Queue import Queue

# import local package
from premaidai_ros_bridge.client import Controller
from premaidai_ros_bridge.srv import MotionRequest

# import ros related packages
import rospy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse

class RosBridgeNode(object):
    def __init__(self):
        rospy.init_node('joint_state_publisher')

        # create ros service interfaces
        rospy.Service('motion_request', MotionRequest, self._motion_request)
        rospy.Service('power_off_request', Trigger, self._servo_off_request)

        # create ros topic interfaces
        rospy.Subscriber('joint_command', JointState, self._set_joint_trajectory)
        self._joint_pub = rospy.Publisher('joint_states', JointState, queue_size=10)

        # launch controller
        port_name = rospy.get_param("~serial_port", "/dev/rfcomm0")
        file_name = rospy.get_param("~config_file", "NONE")
        publish_joint_state = rospy.get_param("~publish_joint_state", "NONE")
        self._controller = Controller('/dev/rfcomm0', 1, file_name, self._joint_state_callback, publish_joint_state)

        # get param
        self._publish_joint_states = rospy.get_param("~publish_joint_states", True)

    def _joint_state_callback(self, joint_angle_dict):
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        for name, angle in joint_angle_dict.items():
            joint_state.name.append(name)
            joint_state.position.append(angle)

        if self._publish_joint_states:
            self._joint_pub.publish(joint_state)

    def _motion_request(self, request):
        result = self._controller.execute_motion(request.motion_command)
        return result

    def _servo_off_request(self, request):
        result = self._controller._servo_off_request()
        response = TriggerResponse(result, "hogehoge")
        return response

    def _set_joint_trajectory(self, request):
        names = self._controller.get_joint_names()
        if set(request.name) != set(names):
            rospy.logerr("illigal joint name")
            return

        if len(request.name) != len(request.position):
            rospy.logerr("invalid position size")
            return

        joint_command = dict(zip(request.name, request.position))
        self._controller.set_joint_positions(joint_command)

    def run(self):
        self._controller.start()
        rate = rospy.Rate(10)

        # create servo off command
        if rospy.get_param('~servo_off_mode', False):
            self._controller.power_off()

        try:
            while not rospy.is_shutdown():
                rate.sleep()

        except KeyboardInterrupt:
            pass

        except Exception as err:
            rospy.logerr("Exception occured. err = {}".format(err))

        # shutdown interfaes
        self._controller.shutdown()


def main():
    node = RosBridgeNode()
    node.run()


if __name__ == '__main__':
    main()
