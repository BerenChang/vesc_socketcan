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

    # os.system('sudo ip link set can1 up type can bitrate 1000000 dbitrate 8000000 restart-ms 1000 berr-reporting on fd on')
    # os.system('sudo ip link set can0 up type can bitrate 500000')
    # os.system('sudo ip link set can1 up type can bitrate 500000')

    joystick_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("teleop_twist_joy"), '/launch', '/teleop-launch.py']),
        launch_arguments={
            'joy_config': 'xbox'
        }.items(),
    )
    
    joy_converter_node = Node(
        name='joy_converter_node',
        package='vesc_socketcan',
        executable='joy_converter_node',
        output='screen'
    )

    vesc_can_node = Node(
        name='vesc_socketcan_node',
        package='vesc_socketcan',
        executable='vesc_socketcan_node',
        output='screen'
    )

    vesc_can_node2 = Node(
        name='vesc_socketcan_node2',
        package='vesc_socketcan',
        executable='vesc_socketcan_node2',
        output='screen'
    )

    return LaunchDescription([
        joystick_node,
        joy_converter_node,
        vesc_can_node,
        # vesc_can_node2
    ])
