from launch import LaunchDescription
import launch.actions

def generate_launch_description():
    return LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-s mcap', '-d 5', '-o my_bag'],
        )
    ])
    
