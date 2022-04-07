"""
# Copyright (c) 2012-2021 Murilo Marques Marinho
#
#    This file is part of sas_robot_driver.
#
#    sas_robot_driver is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    sas_robot_driver is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with sas_robot_driver.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilo@nml.t.u-tokyo.ac.jp
#
# ################################################################
"""
import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
import sas_conversions as rc


class RobotDriverInterface:

    def __init__(self, node_prefix):

        self.enabled_ = False
        self.joint_positions_ = None
        self.joint_limits_min_ = None
        self.joint_limits_max_ = None
        self.reference_frame_ = None

        self.publisher_target_joint_positions_ = rospy.Publisher(node_prefix + "set/target_joint_positions",
                                                                 Float64MultiArray,
                                                                 queue_size=1)

        self.subscriber_joint_states_ = rospy.Subscriber(node_prefix + "get/joint_states", JointState,
                                                         self._callback_joint_states)
        self.subscriber_joint_limits_min_ = rospy.Subscriber(node_prefix + "get/joint_positions_min",
                                                             Float64MultiArray, self._callback_joint_limits_min)
        self.subscriber_joint_limits_max_ = rospy.Subscriber(node_prefix + "get/joint_positions_max",
                                                             Float64MultiArray, self._callback_joint_limits_max)

    def send_target_joint_positions(self, target_positions):
        msg = Float64MultiArray(data=target_positions)
        self.publisher_target_joint_positions_.publish(msg)

    def get_joint_positions(self):
        if self.joint_positions_ is None:
            raise Exception("[{}]::Tried to obtain uninitialized get_joint_positions()".format(rospy.get_name()))
        return self.joint_positions_

    def get_joint_limits(self):
        if self.joint_limits_min_ is None:
            raise Exception("[{}]::Tried to obtain uninitialized joint_limits_min_".format(rospy.get_name()))
        if self.joint_limits_max_ is None:
            raise Exception("[{}]::Tried to obtain uninitialized joint_limits_max_".format(rospy.get_name()))
        return self.joint_limits_min_, self.joint_limits_max_

    def get_reference_frame(self):
        if self.reference_frame_ is None:
            raise Exception("[{}]::Tried to obtain uninitialized reference_frame_".format(rospy.get_name()))
        return self.reference_frame_

    def is_enabled(self):
        if (self.joint_positions_ is not None) and \
                (self.joint_limits_min_ is not None) and \
                (self.joint_limits_max_ is not None):
            return True

    def _callback_joint_states(self, msg):
        self.joint_positions_ = msg.position

    def _callback_joint_limits_min(self, msg):
        self.joint_limits_min_ = msg.data

    def _callback_joint_limits_max(self, msg):
        self.joint_limits_max_ = msg.data

