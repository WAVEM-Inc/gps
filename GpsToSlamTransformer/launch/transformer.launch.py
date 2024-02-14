import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    package_name: str = 'gps_to_slam_transformer'
    package_shared_directory: str = get_package_share_directory('gps_to_slam_transformer')
    executable_name: str = 'transformer'
    
    transformer_node = Node(
        package=package_name,
        executable=executable_name,
        name=executable_name,
        arguments=['--ros-args','--enclave','/transformer'],
        output='screen',
        parameters=[]
    )
    
    ld.add_action(transformer_node)
    
    return ld
