
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
    
    DeclareLaunchArgument('mode',
                          choices=['dev', 'prod'],
                          default_value='dev',
                          description="Execution in production or in dev. environment"),
    
    DeclareLaunchArgument('init_topic',
                          default_value='World (Default init topic)',
                          description="Topic that be published in the first 10 secs."),
    
    DeclareLaunchArgument('params_file',
                          default_value=PathJoinSubstitution([get_package_share_directory('minimal_publisher_extended'), 
                                                              'params', 'publisher_param.yaml']),
                          description="Parameter file path"),
]

def generate_launch_description():
    
    # directory 
    pkg_rclpy_minimal_publisher = get_package_share_directory('minimal_publisher_extended')
    
    # args: declared launch parameters
    arg_mode = LaunchConfiguration('mode')
    arg_init_topic = LaunchConfiguration('init_topic')
    arg_params_file = LaunchConfiguration('params_file')
    
    # talker node
    talker_node = Node(
        package="minimal_publisher_extended",
        executable="publisher_with_param",
        parameters=[{
            'init_topic': arg_init_topic
        }, arg_params_file]
    )
    
    ld = LaunchDescription(ARGUMENTS)    
    ld.add_action(talker_node)
    
    return ld
