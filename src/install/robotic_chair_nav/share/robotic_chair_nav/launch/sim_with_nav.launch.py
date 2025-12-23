from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    desc_share = FindPackageShare("robotic_chair_description").find("robotic_chair_description")
    urdf_file = os.path.join(desc_share, "urdf", "chair.urdf")

    return LaunchDescription([
        ExecuteProcess(
            cmd=["gazebo", "--verbose", "-s", "libgazebo_ros_factory.so"],
            output="screen"
        ),
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-file", urdf_file, "-entity", "robotic_chair"],
            output="screen"
        ),
        Node(
            package="robotic_chair_nav",
            executable="obstacle_stop",
            output="screen",
            parameters=[{
                "forward_speed": 0.25,
                "stop_distance": 0.6,
                "scan_topic": "/scan",
                "cmd_vel_topic": "/cmd_vel",
            }],
        ),
    ])
