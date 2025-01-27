from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
import subprocess
import time

def generate_launch_description():
    container = ComposableNodeContainer(
        name='pcl_ros_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='pcl_ros',
                plugin='pcl_ros::PointCloudToPCD',
                name='pointcloud_to_pcd_node',
                parameters=[
                    {
                        'filename_format': 'zed_cloud_%d.pcd',
                        'binary': True,
                        'compressed': False,
                        'trigger': True,  # Only save on trigger
                    }
                ],
                remappings=[
                    ('/input', '/cloud_in_downsampled') 
                ]
            )
        ],
        output='screen',
    )

    # Node to trigger the saving service after a delay
    # trigger_client = TimerAction(
    #     period=5.0,  # Wait 5 seconds to ensure the node is ready
    #     actions=[
    #         LogInfo(msg="Triggering PointCloudToPCD save"),
    #         Node(
    #             package='ros2cli',
    #             executable='ros2cli',
    #             arguments=[
    #                 'service', 'call', 
    #                 '/pointcloud_to_pcd_node/trigger', 
    #                 'std_srvs/srv/Trigger', 
    #                 '{}'
    #             ],
    #             output='screen'
    #         )
    #     ]
    # )

    return LaunchDescription([container])

time.sleep(10) 
process_trigger = subprocess.Popen(['exec ros2 service call /pointcloud_to_pcd_node/trigger std_srvs/srv/Trigger'], stdout=subprocess.PIPE, stderr=subprocess.PIPE,shell=True)

# ros2 service call /pointcloud_to_pcd_node/trigger std_srvs/srv/Trigger
