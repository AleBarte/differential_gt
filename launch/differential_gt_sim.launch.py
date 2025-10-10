from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # All the launched nodes must have the same publishing rate
    publishing_rate = 500.0

    return LaunchDescription([
        # Declare launch argument for algorithm
        DeclareLaunchArgument(
            'algorithm',
            default_value='dorigo',
            description='Algorithm type for differential GT (options: manual, dorigo, pedrocchi)'
        ),

        Node(
            package = "compliance_controller",
            executable = "virtual_wrench_commander",
            name = "virtual_wrench_commander",
            output = "screen",
            parameters = [
                {"publishing_rate": publishing_rate},
                {"exclude_acs": False},
                {"use_safety_filter": False},
                {"wrench_from_joystick_topic": "/differential_gt/wrench_from_ho"}
            ]
        ),

        Node(
            package = "differential_gt",
            executable = "differential_gt",
            name = "differential_gt",
            output = "screen",
            parameters = [
                {"publishing_rate": publishing_rate},
                {"algorithm": LaunchConfiguration('algorithm')}
            ]
        )
    ])