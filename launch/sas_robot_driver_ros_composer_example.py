from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sas_robot_driver',
            executable='sas_robot_driver_ros_example',
            name='robot_1'
        ),
        Node(
            package='sas_robot_driver',
            executable='sas_robot_driver_ros_example',
            name='robot_2'
        ),
        Node(
            package='sas_robot_driver',
            executable='sas_robot_driver_ros_composer_node',
            name='robot_composed',
            parameters=[{
                "use_real_robot": True,
                "use_coppeliasim": False,
                "robot_driver_client_names": ["robot_1","robot_2"],
                "override_joint_limits_with_robot_parameter_file": False,
                "thread_sampling_time_sec": 0.01
            }]
        )
    ])

