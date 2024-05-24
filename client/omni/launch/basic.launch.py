from launch_ros.actions import Node
from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.event_handlers import (
    OnExecutionComplete, OnProcessExit, OnProcessIO,
    OnProcessStart, OnShutdown)

from launch.actions import (
    RegisterEventHandler, LogInfo, ExecuteProcess)

def generate_launch_description():
    
    lidar_launch_path = PathJoinSubstitution(
        [FindPackageShare('sllidar_ros2'), 'launch', 'sllidar_s1_launch.py']
    )
    
    node2 = Node(
        package="omni",
        executable="esp32_bridge",
        # output='screen'  # Display output in the terminal
    )

    node3 = Node(
        package="omni",
        executable="odom_gpt4o",
        # output='screen'  # Display output in the terminal
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_path),
        ),
        node2,
        RegisterEventHandler(
            OnProcessStart(
                target_action=node2,
                on_start=[
                    LogInfo(msg='esp32 node started'),
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=node3,
                on_start=[
                    LogInfo(msg='odom started'),
                ]
            )
        ),
        node3,
    ])
