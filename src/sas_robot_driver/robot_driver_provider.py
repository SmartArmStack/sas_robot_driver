"""
# Copyright (c) 2012-2022 Murilo Marques Marinho
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
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import sas_conversions as rc


class RobotDriverProvider:

    def __init__(self, node_prefix):
        # Regular Members
        self.target_joint_positions_ = None
        
        # ROS Publishers
        self.publisher_joint_states_ = rospy.Publisher(node_prefix + "get/joint_states",
                                                       JointState,
                                                       queue_size=1)
        self.publisher_joint_limits_min_ = rospy.Publisher(node_prefix + "get/joint_positions_min",
                                                           Float64MultiArray,
                                                           queue_size=1)
        self.publisher_joint_limits_max_ = rospy.Publisher(node_prefix + "get/joint_positions_max",
                                                           Float64MultiArray,
                                                           queue_size=1)
        self.publisher_reference_frame_ = rospy.Publisher(node_prefix + "get/reference_frame",
                                                          PoseStamped,
                                                          queue_size=1)
        
        # ROS Subscribers
        self.subscriber_target_joint_positions_ = rospy.Subscriber(node_prefix + "set/target_joint_positions",
                                                                   Float64MultiArray,
                                                                   self._callback_target_joint_positions)

    def get_target_joint_positions(self):
        if self.target_joint_positions_ is None:
            raise Exception("Tried to obtain uninitialized get_target_joint_positions()")
        return self.target_joint_positions_

    def send_joint_positions(self, joint_positions):
        msg = JointState(position=joint_positions)
        self.publisher_joint_states_.publish(msg)

    def send_joint_limits(self, joint_limits):
        msg_min = Float64MultiArray(data=joint_limits[0])
        self.publisher_joint_limits_min_.publish(msg_min)
        msg_max = Float64MultiArray(data=joint_limits[1])
        self.publisher_joint_limits_max_.publish(msg_max)

    def send_reference_frame(self, reference_frame):
        self.publisher_reference_frame_.publish(rc.dq_to_geometry_msgs_pose_stamped(reference_frame))

    def _callback_target_joint_positions(self, msg):
        self.target_joint_positions_ = msg.data

    def is_enabled(self):
        return (self.target_joint_positions_ is not None)
