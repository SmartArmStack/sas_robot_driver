# Copyright (c) 2020-2022
# Murilo Marques Marinho at the University of Tokyo.
# This software can be used for Research Purposes only.
# For commercial purposes, contact the author.
# Author: Murilo M. Marinho, email: murilo@g.ecc.u-tokyo.ac.jp
import time
from dqrobotics import *  # Despite what PyCharm might say, this is very much necessary or DQs will not be recognized
import rospy
from sas_robot_driver import RobotDriverInterface, RobotDriverProvider

rospy.init_node('my_node_name', disable_signals=True)
try:
    # Initialize the RobotDriveProvider
    rdp = RobotDriverProvider('my_test_robot')

    # Initialize the RobotDriverInterface
    rdi = RobotDriverInterface('my_test_robot')

    # Wait for RobotDriverInterface to be enabled
    while not rdi.is_enabled():
        time.sleep(0.1)
        # Send positions and limits from the RobotDriverProvider to the RobotDriverInterface
        # RobotDriverInterface will be enabled when those values are received
        rdp.send_joint_states([1, 2, 3, 4], [5, 6, 7, 8], [9, 10, 11, 12])
        rdp.send_joint_limits(([-5, -5, -5, -5], [5, 5, 5, 5]))

    # Read the values sent by the RobotDriverProvider
    print(rdi.get_joint_positions())
    print(rdi.get_topic_prefix())

except KeyboardInterrupt:
    print("Interrupted by user")
