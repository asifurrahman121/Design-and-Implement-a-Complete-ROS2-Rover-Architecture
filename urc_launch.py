from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='urc_rover',
            executable='mission_manager',
            name='mission_manager'
        ),
        Node(
            package='urc_rover',
            executable='delivery_action_server',
            name='delivery_server'
        ),
        Node(
            package='urc_rover',
            executable='delivery_action_client',
            name='delivery_client'
        ),
    ])

