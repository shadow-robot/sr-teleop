#!/usr/bin/env python
#
# Copyright (C) 2019 Shadow Robot Company Ltd - All Rights Reserved.
# Proprietary and Confidential. Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import rospy
import rospkg
import argparse
import actionlib
import tty
import sys
import tf
import tf2_ros
import yaml
import geometry_msgs.msg
import termios
from std_srvs.srv import Trigger, TriggerRequest
from threading import Thread, Lock
from sr_vive_tracking.msg import ControllerEvent
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class VRCodes:
    k_EButton_ApplicationMenu = 1
    k_EButton_SteamVR_Trigger = 33
    VREvent_ButtonPress = 200
    VREvent_ButtonUnpress = 201
    TrackedDeviceClass_Controller = 2

class CybergloveMock(object):
    def __init__(self, cycle_hand_positions=True):
        self._cycle_hand_positions = cycle_hand_positions

        self._button_publisher = rospy.Publisher('sr_vive_controller_events', ControllerEvent, queue_size=5)
        self._hand_traj_client = actionlib.SimpleActionClient('/sr_teleop_vive_cyberglove_trajectory_controller' +
                                                              '/follow_joint_trajectory',
                                                              FollowJointTrajectoryAction)

    def open_hand(self):
        CONST_OPEN_HAND_JOINT_VALUES = [0] * 24

        self._send_goal_to_hand(CONST_OPEN_HAND_JOINT_VALUES)

    def pack_hand(self):
        CONST_PACK_HAND_JOINT_VALUES = [1.571, 1.57, 1.571,
                                        0.0, 1.57, 1.571,
                                        1.571, 0.0, 0.0,
                                        1.571, 1.571, 1.571,
                                        0.0, 1.571, 1.571,
                                        1.571, 0.0, 0.0,
                                        0.0, 0.0, 0.0,
                                        0.0, 0.14, 0.0]

        self._send_goal_to_hand(CONST_PACK_HAND_JOINT_VALUES)

    def _send_goal_to_hand(self, hand_positions):
        self._hand_traj_client.wait_for_server()
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.header.stamp = rospy.Time.now()
        goal.trajectory.joint_names = ["rh_FFJ1", "rh_FFJ2", "rh_FFJ3", "rh_FFJ4", "rh_MFJ1", "rh_MFJ2",
                                       "rh_MFJ3", "rh_MFJ4", "rh_RFJ1", "rh_RFJ2", "rh_RFJ3", "rh_RFJ4",
                                       "rh_LFJ1", "rh_LFJ2", "rh_LFJ3", "rh_LFJ4", "rh_LFJ5", "rh_THJ1",
                                       "rh_THJ2", "rh_THJ3", "rh_THJ4", "rh_THJ5", "rh_WRJ1", "rh_WRJ2"]
        joint_traj_point = JointTrajectoryPoint()

        joint_traj_point.positions = hand_positions

        joint_traj_point.time_from_start = rospy.Duration(0.1)
        goal.trajectory.points.append(joint_traj_point)
        self._hand_traj_client.send_goal(goal)

    def _publish_button_event(self, device_id, button_id, event_id, device_type, device_index):
        controller_event = ControllerEvent()
        controller_event.header.stamp = rospy.Time.now()
        controller_event.device_id = device_id
        controller_event.button_id = button_id
        controller_event.event_id = event_id
        controller_event.device_type = device_type
        controller_event.device_index = device_index
        self._button_publisher.publish(controller_event)

    def trigger_press(self):
        self._publish_button_event(0, VRCodes.k_EButton_SteamVR_Trigger, VRCodes.VREvent_ButtonPress,
                                   VRCodes.TrackedDeviceClass_Controller, 0)
    def trigger_release(self):
        self._publish_button_event(0, VRCodes.k_EButton_SteamVR_Trigger, VRCodes.VREvent_ButtonUnpress,
                                   VRCodes.TrackedDeviceClass_Controller, 0)

    def run(self):
        if self._cycle_hand_positions:
            while not rospy.is_shutdown():
                #self.trigger_press()
                self.pack_hand()
                rospy.loginfo("SPAM")
                rospy.sleep(3)
                self.open_hand()
                rospy.loginfo("SPAM")
                rospy.sleep(3)

            #self.trigger_release()


if __name__ == "__main__":
    rospy.init_node('cyberglove_mock_node')

    cyberglove_mock = CybergloveMock()
    cyberglove_mock.run()
