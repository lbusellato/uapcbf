from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Allow turning on debug mode at launch time
    declare_debug_arg = DeclareLaunchArgument("debug", default_value="false", description="Run in debug/mockup mode")

    nn_forecasting_node = Node(
        package="collaborice_forecasting_node",
        executable="nn_node",  # <-- this must match your entry_point name
        name="nn_node",
        output="screen",
        parameters=[{"debug": LaunchConfiguration("debug")}],
    )

    return LaunchDescription([declare_debug_arg, nn_forecasting_node])
