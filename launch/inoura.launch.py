from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="px4_driver",
            executable="px4_driver_node",
            output="screen",
            # Launch node with sudo rights
            shell=True,
            prefix=["sudo -E env \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\"  bash -c "],
            emulate_tty=True,
        ),
        Node(
            package="px4_driver",
            executable="inoura_connection",
            output="screen",
            # Launch node with sudo rights
            shell=True,
            prefix=["sudo -E env \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\"  bash -c "],
            emulate_tty=True,
        ),
        Node(
            package="px4_driver",
            executable="trajectory_manager",
            output="screen",
            # Launch node with sudo rights
            shell=True,
            prefix=["sudo -E env \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\"  bash -c "],
            emulate_tty=True,
        )
    ])