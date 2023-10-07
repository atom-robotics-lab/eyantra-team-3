from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    controller_node = Node(
        package="<your_package_name>",           # Enter the name of your ROS2 package
        executable="<your_executable_name>",    # Enter the name of your executable
    )
    service_node = Node(
        package="<your_package_name>",           # Enter the name of your ROS2 package
        executable="<your_executable_name>",    # Enter the name of your executable
    )
