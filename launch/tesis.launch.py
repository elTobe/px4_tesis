from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=["MicroXRCEAgent", "--serial", "/dev/ttyAMA0", "-b", "921600"],
            output="screen"
        ),
        Node(
            package="px4_driver",
            executable="px4_driver_node",
            output="screen"
        ),
        Node(
            package="px4_driver",
            executable="trajectory_manager",
            output="screen"
        )
    ])