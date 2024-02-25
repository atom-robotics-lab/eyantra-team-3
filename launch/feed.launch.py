from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import IncludeLaunchDescription ,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution,LaunchConfiguration, PythonExpression
import os
from ament_index_python.packages import get_package_share_directory,get_package_prefix

def generate_launch_description():

    usb = get_package_share_directory('usb_cam')
    img = get_package_share_directory('image_proc')
    feed = get_package_share_directory('eyantra_hb_task_6')

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(usb, 'launch', 'camera.launch.py'),
        )
    )

    spwan_bot=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(img, 'launch', 'image_proc.launch.py'),
        )
    )

    feedback = Node(
            package = "eyantra_hb_task_6",
            executable = 'feedback.py',
            name = 'feedback_node'
        )
    return LaunchDescription([
        world,
        spwan_bot,
        feedback
        ])