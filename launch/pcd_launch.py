import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('pcd'),
        'config',
        'viz.yaml'
    )

    # Declare launch arguments
    visualization_config_file = LaunchConfiguration('visualization_config')
    declare_visualization_config_file_cmd = DeclareLaunchArgument(
        'visualization_config',
        default_value=os.path.join(
            get_package_share_directory('pcd'),'config','viz.yaml'),
        description='Full path to the Gridmap visualization config file to use')

    pcd_node = Node(
        package='pcd',
        executable='pcd_process',
        name='pcd_process',
        output='screen'
    )

    grid_map_visualization_node = Node(
        package='grid_map_visualization',
        executable='grid_map_visualization',
        name='grid_map_visualization',
        output='screen',
        parameters=[visualization_config_file]
    )
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_livox',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'livox_frame'],
        output='screen'
    )
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    ld = LaunchDescription()

    # Add launch arguments to launch description
    ld.add_action(declare_visualization_config_file_cmd)

    # Add node action to description
    ld.add_action(pcd_node)
    ld.add_action(grid_map_visualization_node)
    ld.add_action(tf_node)
    ld.add_action(rviz2_node)
    
    return ld
