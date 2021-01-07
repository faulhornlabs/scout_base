from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
         DeclareLaunchArgument(
            'is_scout_mini',
            default_value='true',
            description='Scout mini values'),

        DeclareLaunchArgument(
            'port_name',
            default_value='can0',
            description='Can interface name'),
        
        DeclareLaunchArgument(
            'simulated_robot',
            default_value='false',
            description='Robot simulation'),

        DeclareLaunchArgument(
            'odom_frame',
            default_value='odom_scout',
            description='Odometry frame name'),

        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Odometry frame name'),

        DeclareLaunchArgument(
            'odom_topic_name',
            default_value='odom_scout',
            description='Odometry frame name'),

        Node(
            package='scout_base',
            executable='scout_base_node',
            arguments=[('--ros-args --log-level INFO')],
            name='scout_base',
            parameters=[
                {'is_scout_mini': LaunchConfiguration('is_scout_mini')},
                {'port_name': LaunchConfiguration('port_name')},
                {'simulated_robot': LaunchConfiguration('simulated_robot')},
                {'odom_frame': LaunchConfiguration('odom_frame')},
                {'base_frame': LaunchConfiguration('base_frame')},
                {'odom_topic_name': LaunchConfiguration('odom_topic_name')},
            ]
        ),
    ])
