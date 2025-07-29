from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # LeapMotion streamer node
    leap_streamer_node = Node(
        package="collaborice_forecasting_node", 
        executable="leap_forecasting", 
        name="collaborice_forecasting_node", 
        output="screen"
    )

    return LaunchDescription([leap_streamer_node])
