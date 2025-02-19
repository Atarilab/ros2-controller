from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_controller',
            executable='state_viewer_node',
            name='state_viewer_node'
        ),
        Node(
            package='robot_controller',
            executable='controller_node',
            name='controller_node'
        ),
        Node(
            package='robot_controller',
            executable='simulator_node',
            name='simulator_node'
        ),
    ])