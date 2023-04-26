# Copyright (c) 2020-2022
# Murilo Marques Marinho at the University of Tokyo.
# This software can be used for Research Purposes only.
# For commercial purposes, contact the author.
# Author: Murilo M. Marinho, email: murilomarinho@ieee.org
import time
from dqrobotics import *  # Despite what PyCharm might say, this is very much necessary or DQs will not be recognized

from sas_common import rclcpp_init, rclcpp_Node, rclcpp_spin_some, rclcpp_shutdown
from sas_robot_driver import RobotDriverClient, RobotDriverServer

def main(args=None):
    try:
        # rclpy.init(args=args)
        rclcpp_init()

        node = rclcpp_Node("my_test_robot_node")

        # Initialize the RobotDriverServer
        rdp = RobotDriverServer(node,'my_test_robot')

        # Initialize the RobotDriverClient
        rdi = RobotDriverClient(node,'my_test_robot')

        # Wait for RobotDriverClient to be enabled
        while not rdi.is_enabled():
            rclcpp_spin_some(node)
            time.sleep(0.1)
            # Send positions and limits from the RobotDriverServer to the RobotDriverClient
            # RobotDriverClient will be enabled when those values are received
            rdp.send_joint_states([1, 2, 3, 4], [5, 6, 7, 8], [9, 10, 11, 12])
            rdp.send_joint_limits(([-5, -5, -5, -5], [5, 5, 5, 5]))

        # Read the values sent by the RobotDriverServer
        print(rdi.get_joint_positions())
        print(rdi.get_topic_prefix())

        rclcpp_shutdown()

    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print("Unhandled excepts", e)


if __name__ == '__main__':
    main()
