from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    world_file = os.path.join(
        FindPackageShare("simple_world").find("simple_world"),
        "worlds",
        "robotic_chair_worlds.sdf"
    )

    robot_urdf = os.path.join(
        FindPackageShare("robotic_chair_description").find("robotic_chair_description"),
        "urdf",
        "chair.urdf"
    )

    return LaunchDescription([

        # ---------------- GAZEBO ----------------
        ExecuteProcess(
            cmd=[
                "gazebo",
                "--verbose",
                world_file,
                "-s", "libgazebo_ros_factory.so"
            ],
            output="screen"
        ),

        # ---------------- SPAWN ROBOT ----------------
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity", "robotic_chair",
                "-file", robot_urdf
            ],
            output="screen"
        ),

        # ---------------- KEYBOARD CONTROL NODE ----------------
        Node(
            package="robotic_chair_nav",
            executable="keyboard_control",
            name="keyboard_control",
            output="screen",
            parameters=[
                {"linear_speed": 0.3},
                {"angular_speed": 0.5},
                {"stop_distance": 0.1},
                {"scan_topic": "/scan"},
                {"cmd_vel_topic": "/cmd_vel"},
                {"front_half_angle_deg": 20.0}
            ]
        ),
    ])