from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='filter_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='pcl_ros',
                    plugin='pcl_ros::VoxelGrid',
                    name='voxelgrid',
                    # remappings=[('/input', '/input_map'),('/output', '/output_map')],
                    # remappings=[('/input', '/cloud_in'),('/output', '/cloud_in_downsampled')],
                    remappings=[('/input', '/zed/zed_node/point_cloud/cloud_registered'),('/output', '/cloud_in_downsampled')],
                    # parameters=[{'filter_field_name': 'z', 'filter_limit_min': 0.3, 'filter_limit_max': 3.5, 'filter_limit_negative': False, 'leaf_size': 0.2}],
                    parameters=[{'filter_field_name': 'z', 'filter_limit_min': -0.8, 'filter_limit_max': 2.5, 'filter_limit_negative': False, 'leaf_size': 0.2}],
                ),
                ComposableNode(
                    package='pcl_ros',
                    plugin='pcl_ros::VoxelGrid',
                    name='voxelgrid_ground_removal',
                    # remappings=[('/input', '/input_map'),('/output', '/output_map')],
                    remappings=[('/input', '/cloud_transformed'),('/output', '/pc2_filtered')],
                    # parameters=[{'filter_field_name': 'z', 'filter_limit_min': 0.3, 'filter_limit_max': 3.5, 'filter_limit_negative': False, 'leaf_size': 0.2}],
                    parameters=[{'filter_field_name': 'z', 'filter_limit_min': -0.8, 'filter_limit_max': 2.5, 'filter_limit_negative': False, 'leaf_size': 0.2}],
                ),
            ],
            output='both',
    )


    return LaunchDescription([container, 
        Node(
            package='pc2_to_grid',
            executable='pc2transform',
            name='pc2transform',
            output='screen',
            parameters=[
                {"input_topic": "/cloud_in_downsampled"},
                {"output_topic": "/cloud_transformed"},
                {"target_frame": "map"},
            ]
        ),
        Node(
            package='pc2_to_grid',
            executable='pc2grid',
            name='pc2grid',
            output='screen',
            parameters=[
                {"pc2_topic": "/pc2_filtered"},
                {"odom_topic": "/zed/zed_node/odom"},
                {"occupancy_grid_topic": "/occupancy_grid_scan"},
                {"frame_id": "map"},
                {"grid_resolution": 0.1},
                {"grid_width": 1000},
                {"grid_height": 1000},
                {"max_range": 10.0},
            ]
        )
    ])