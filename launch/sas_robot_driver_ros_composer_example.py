from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sas_robot_driver',
            executable='sas_robot_driver_ros_example',
            name='robot_1',
            parameters=[{
                "robot_name": "robot_1",
                "joint_limits_min": [-1., -2., -3., -4.],
                "joint_limits_max": [1., 2., 3., 4],
                "initial_joint_positions": [0., 0., 0., 0.],
                "thread_sampling_time_sec": 0.001
            }]
        ),
        Node(
            package='sas_robot_driver',
            executable='sas_robot_driver_ros_example',
            name='robot_2',
            parameters=[{
                "robot_name": "robot_2",
                "joint_limits_min": [-5., -6., -7., -8., -9., -10.],
                "joint_limits_max": [5., 6., 7., 8., 9., 10.],
                "initial_joint_positions": [0., 0., 0., 0., 0., 0.],
                "thread_sampling_time_sec": 0.001
            }]
        ),
        Node(
            package='sas_robot_driver',
            executable='sas_robot_driver_ros_composer_node',
            name='robot_composed',
            parameters=[{
                "use_real_robot": True,
                "use_coppeliasim": False,
                "robot_driver_client_names": ["robot_1", "robot_2"],
                "override_joint_limits_with_robot_parameter_file": False,
                "thread_sampling_time_sec": 0.01
            }]
        )
    ])
