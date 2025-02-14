from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_controller',
            executable='state_publisher_node',
            name='state_publisher_node'
        ),
        Node(
            package='robot_controller',
            executable='action_publisher_node',
            name='action_publisher_node'
        ),
        Node(
            package='robot_controller',
            executable='controller_node',
            name='controller_node'
        ),
    ])