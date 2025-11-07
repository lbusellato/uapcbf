from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # LeapMotion streamer node
    leap_streamer_node = Node(
        package="leap_forecasting", 
        executable="leap_forecasting", 
        name="leap_forecasting", 
        output="screen"
    )

    return LaunchDescription([leap_streamer_node])
