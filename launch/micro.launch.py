import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='node_8887',
            arguments=['udp4', '--port', '8887'],
            parameters=[{'log_level': '4'}]
        ),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='node_8888',
            arguments=['udp4', '--port', '8888', 'log_level','4']
        ),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='node_8889',
            arguments=['udp4', '--port', '8889'],
            parameters=[{'log_level': '4'}]
        )
    ])

