import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, Shutdown


def generate_launch_description():
    output_file = os.path.expanduser("/home/orin/my_3dmap.pcd")

    return LaunchDescription([
        Node(
            package='pcl_ros',
            executable='pointcloud_to_pcd',
            name='pointcloud_saver',
            output='screen',
            parameters=[{"filename": output_file}],
            remappings=[
                # ("/input", "/bonxai_point_cloud_centers")
                ("/input", "/zed/zed_node/mapping/fused_cloud")
            ]
        ),

        TimerAction(
            period=2.0,
            actions=[Shutdown()]
        )
    ])
