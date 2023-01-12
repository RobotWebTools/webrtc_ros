from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_launch_arg = DeclareLaunchArgument(
        'port',
        default_value='9090'
    )    
    return LaunchDescription([
        port_launch_arg,
        Node(
            package='webrtc_ros',
            executable='webrtc_ros_server_node',
            name='webrtc_server',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port')
           }]
        ),    
    ])