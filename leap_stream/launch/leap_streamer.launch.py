from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='desk',
        description='Name to associate to the topic'
    )

    tracking_mode_arg = DeclareLaunchArgument(
        'tracking_mode',
        default_value='Desktop',
        description='Tracking mode (Desktop or HMD)'
    )

    forecasting_method_arg = DeclareLaunchArgument(
        'forecasting_method', default_value='nn', description='Forecasting method')

    fake_data_arg = DeclareLaunchArgument(
        'fake_data', default_value='False', description='Use prerecorded data instead of connecting to a Leap camera')
    fake_data = LaunchConfiguration('fake_data')
    leap_streamer_node = Node(
        package='leap_stream',
        executable='leap_streamer',
        name='leap_streamer',
        parameters=[{
            'camera_name': LaunchConfiguration('camera_name'),
            'tracking_mode': LaunchConfiguration('tracking_mode'),
            'fake_data': LaunchConfiguration('fake_data'),
            'forecasting_method': LaunchConfiguration('forecasting_method')
        }],
        condition=IfCondition(
            PythonExpression([
                fake_data,
                ' == False',
            ])),
        output='screen',
        emulate_tty=True
    )
    fake_leap_streamer_node = Node(
        package='leap_stream',
        executable='fake_leap_streamer',
        name='fake_leap_streamer',
        parameters=[{
            'camera_name': LaunchConfiguration('camera_name'),
            'forecasting_method': LaunchConfiguration('forecasting_method')
        }],
        condition=IfCondition(
            PythonExpression([
                fake_data,
                ' == True',
            ])),
        output='screen',
        emulate_tty=True
    )

    leap_fusion_node = Node(
        package = 'leap_stream',
        executable = 'leap_fusion',
        name = 'leap_fusion',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        camera_name_arg,
        fake_data_arg,
        tracking_mode_arg,
        forecasting_method_arg,
        leap_streamer_node,
        fake_leap_streamer_node,
        leap_fusion_node
    ])