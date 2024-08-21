import os
from struct import pack

from setuptools import Command

from ament_index_python import get_package_share_directory, get_package_share_path
from launch_ros.actions import Node
import launch
import launch.launch_description_sources
import launch.substitutions
import launch_ros
import launch_ros.substitutions
import yaml


def generate_launch_description():

    turtlebot2_bringup_package = get_package_share_directory(
        'turtlebot2_bringup')

    base_bringup = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot2_bringup_package,
                'launch/icclab_tb2_bringup.launch.py'
            )
        ),
    )

    camera_bringup = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot2_bringup_package,
                'launch/astra_icclab.launch.py'
            )
        ),
    )

    color_compression = Node(
        package='image_transport',
        executable='republish',
        # namespace=namespace,
        # parameters=[params],
        arguments=['raw', 'compressed'],
        remappings=[('in', '/camera/color/image_raw'),
                    ('out/compressed', '/camera/color/image/compressed')]
        
    )

    depth_compression = Node(
        package='image_transport',
        executable='republish',
        # namespace=namespace,
        # parameters=[params],
        arguments=['raw', 'compressedDepth'],
        remappings=[('in', '/camera/depth/image_raw'),
                    ('out/compressedDepth', '/camera/depth/image/compressedDepth')]
        
    )

    nav2_bringup = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot2_bringup') ,
                'launch/nomap/icclab_tb2_nav2_nomap.launch.py'
            )
        ),
    )

    ld = launch.LaunchDescription()

    # launch.actions.DeclareLaunchArgument(
    #     'open_rviz',
    #     default_value='false',
    #     description='open rviz'),
    ld.add_action(launch.actions.GroupAction([
        base_bringup,
        camera_bringup,
        color_compression,
        depth_compression,
        nav2_bringup
        # rviz_node,
        # ekf_node
    ]))

    return ld
