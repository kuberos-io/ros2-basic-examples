
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # parameter delaration
    init_topic = LaunchConfiguration('init_topic', default='true')
    
    declare_init_topic_cmd = DeclareLaunchArgument(
        'init_topic', 
        default_value='Earth'
    )
    
    # parameter files
    params_file = LaunchConfiguration(
        'params_file', 
        default='publisher_param.yaml')
    publisher_dir = get_package_share_directory('examples_rclpy_minimal_publisher')
    
    config = os.path.join(publisher_dir, 'config', 'publisher_param.yaml')
    
    # config = '/workspace/src/hello_world_module/minimal_publisher/config/publisher_param.yaml'
    config = '/workspace/config/minimal_publisher/publisher_param.yaml'
    
    ld = LaunchDescription()
    talker_node = Node(
        package="examples_rclpy_minimal_publisher",
        executable="publisher_with_param",
        parameters=[{
            'init_topic': init_topic
        }, config]
    )
    
    ld.add_action(declare_init_topic_cmd)
    ld.add_action(talker_node)
    return ld