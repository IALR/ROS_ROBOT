from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Path to your URDF
    desc_share = FindPackageShare("robotic_chair_description").find("robotic_chair_description")
    urdf_file = os.path.join(desc_share, "urdf", "chair.urdf")

    return LaunchDescription([
        # Start Gazebo
        ExecuteProcess(
            cmd=["gazebo", "--verbose", "-s", "libgazebo_ros_factory.so"],
            output="screen"
        ),
        # Spawn robot
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-file", urdf_file, "-entity", "robotic_chair"],
            output="screen"
        ),
        # Move forward node
        
        # Obstacle stop node
        Node(
            package="robotic_chair_nav",
            executable="obstacle_stop_node",
            output="screen",
            parameters=[{
                "forward_speed": 0.25,
                "stop_distance": 0.5,
                "scan_topic": "/scan",
                "cmd_vel_topic": "/cmd_vel",
            }],
        ),
    ])
