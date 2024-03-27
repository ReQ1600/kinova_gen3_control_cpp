from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():
    description = LaunchDescription()

    move_to_point_node = Node(
        package="kinova_gen3_control_cpp",
        executable="arm_move_to_point_server"
    )

    rviz_kinova_simulation_launchfile = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('kinova_gen3_7dof_robotiq_2f_85_moveit_config'),
                'launch',
                'robot.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_ip': 'yyy.yyy.yyy.yyy',
            'use_fake_hardware': 'true'
        }.items()
    )

    description.add_action(rviz_kinova_simulation_launchfile)
    description.add_action(move_to_point_node)

    return description