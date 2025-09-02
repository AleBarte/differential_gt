from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # All the launched nodes must have the same publishing rate
    publishing_rate = 500.0

    return LaunchDescription([
        Node(
            package = "compliance_controller",
            executable = "virtual_wrench_commander",
            name = "virtual_wrench_commander",
            output = "screen",
            parameters = [
                {"publishing_rate": publishing_rate},
                {"exclude_acs": False},
                {"use_safety_filter": False},
                {"wrench_from_joystick_topic": "/differential_gt/wrench_from_ho"},
                {"switch_on_point": 0.9},
                {"switch_off_point": 0.0},
                {"end_effector": "base_link"}
            ]
        )
    ])