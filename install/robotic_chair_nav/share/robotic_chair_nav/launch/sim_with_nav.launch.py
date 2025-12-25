from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Path to world and URDF
    world_file = os.path.join(
        FindPackageShare("simple_world").find("simple_world"),
        "worlds",
        "robotic_chair_worlds.sdf"
    )

    desc_share = FindPackageShare("robotic_chair_description").find(
        "robotic_chair_description"
    )
    urdf_file = os.path.join(desc_share, "urdf", "chair.urdf")

    # Read URDF into a string for robot_state_publisher
    with open(urdf_file, "r") as f:
        robot_description = f.read()

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
                "-file", urdf_file
            ],
            output="screen"
        ),

        # ---------------- KEYBOARD CONTROL ----------------
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

        # ---------------- ROBOT STATE PUBLISHER (for RViz) ----------------
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {"use_sim_time": True},
                {"robot_description": robot_description},
            ],
        ),

        # ---------------- RViz2 ----------------
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            # If you have an RViz config file, add: arguments=["-d", "/path/to/config.rviz"]
        ),
    ])