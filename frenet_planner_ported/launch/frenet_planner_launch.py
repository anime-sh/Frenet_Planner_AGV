import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('frenet_planner_ported'),
        'config',
        'frenet_planner.yaml'
        )
        
    node=Node(
        package = 'frenet_planner_ported',
        name = 'frenet_planner',
        executable = 'frenet_ros_obst',
        parameters = [config]
    )
    ld.add_action(node)
    return ld
 
