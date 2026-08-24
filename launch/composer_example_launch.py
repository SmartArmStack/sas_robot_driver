import os.path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Example composing two example robots and a RobotDriverROSComposer node.

    All parameters are loaded from a YAML configuration file. Pass a
    different file with ``config_file:=/path/to/config.yaml``.
    """
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(get_package_share_directory('sas_robot_driver'), 'config', 'config.yaml')
        ),
        Node(
            package='sas_robot_driver',
            executable='sas_robot_driver_ros_example',
            name='robot_1',
            parameters=[config_file]
        ),
        Node(
            package='sas_robot_driver',
            executable='sas_robot_driver_ros_example',
            name='robot_2',
            parameters=[config_file]
        ),
        Node(
            package='sas_robot_driver',
            executable='sas_robot_driver_ros_composer_node',
            name='robot_composed',
            parameters=[config_file]
        )
    ])
