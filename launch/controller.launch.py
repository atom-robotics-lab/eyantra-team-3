import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='eyantra_hb_task',
        #     executable='feedback.py',
        # ),
        Node(
            package='eyantra_hb_task',
            executable='bot_1_6a_controller.py',
        ),
        Node(
            package='eyantra_hb_task',
            executable='bot_2_6a_controller.py',
        ),
        Node(
            package='eyantra_hb_task',
            executable='bot_3_6a_controller.py',
        ),
        # Node(
        #     package='eyantra_hb_task',
        #     executable='stop_client.py',
        # ),
        Node(
            package='eyantra_hb_task',
            executable='stop_service.py',
        )
        
    ])