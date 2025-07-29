import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    package_name = 'jaka_description'

    use_joint_gui_arg = DeclareLaunchArgument(
        'use_joint_gui', default_value='False', description='Use Joint State Publisher GUI')
    use_joint_gui = LaunchConfiguration("use_joint_gui")
    # Joint state publisher nodes
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(
            PythonExpression([
                use_joint_gui,
                ' == True',
            ])),
        emulate_tty=True
    )

    visualize_leap_arg = DeclareLaunchArgument(
        'visualize_leap', default_value='True', description='Visualize data from a connected Leap camera')
    visualize_leap = LaunchConfiguration("visualize_leap")
    fake_data_arg = DeclareLaunchArgument(
        'fake_data', default_value='True', description='Use prerecorded data instead of connecting to a Leap camera')
    leap_visualizer_node = Node(
        package='leap_stream',
        executable='leap_visualizer',
        name='leap_visualizer',
        condition=IfCondition(
            PythonExpression([
                visualize_leap,
                ' == True',
            ])),
        output='screen',
        parameters=[{'fake_data': LaunchConfiguration("fake_data")}],
        emulate_tty=True
    )

    urdf_file_name = 'jaka.urdf'  
    urdf_file = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        urdf_file_name)

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
        emulate_tty=True
    )

    rviz_config_file = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'display.rviz'
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    leap_streamer_launch = os.path.join(
        get_package_share_directory('leap_stream'),
        'launch',
        'leap_streamer.launch.py'
    )

    leap_fusion = Node(
        package='leap_stream',
        executable='leap_fusion',
        name='leap_fusion',
        output='screen'
    )

    return LaunchDescription([
        use_joint_gui_arg,
        visualize_leap_arg,
        fake_data_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        leap_visualizer_node,
        rviz2_node,
        leap_fusion,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(leap_streamer_launch)
        )
    ])