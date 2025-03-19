from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

	return LaunchDescription([ 
	        Node(
	            package='pc2_to_grid',
	            executable='map_saver',
	            name='map_saver',
	            output='screen',
	            parameters=[
	                {"map_topic": "/occupancy_grid_scan"},
	                {"map_name": "my_map.pgm"},
	            ]
	        )
	    ])