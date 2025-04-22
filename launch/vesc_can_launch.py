import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    joystick_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("teleop_twist_joy"), '/launch', '/teleop-launch.py']),
        launch_arguments={
            'joy_config': 'xbox'
        }.items(),
    )

    vesc_can_node = Node(
        name='vesc_socketcan_node',
        package='vesc_socketcan',
        executable='vesc_socketcan_node',
        output='screen'
    )


    return LaunchDescription([
        joystick_node,
        vesc_can_node
    ])