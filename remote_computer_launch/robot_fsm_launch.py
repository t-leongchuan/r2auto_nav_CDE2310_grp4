from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_fsm_controller',
            executable='supervisor',
            name='supervisor'
        ),
        Node(
            package='robot_fsm_controller',
            executable='exploration',
            name='exploration'
        ),
        Node(
            package='robot_fsm_controller',
            executable='random',
            name='random'
        ),
        Node(
            package='robot_fsm_controller',
            executable='pursuit',
            name='pursuit'
        ),
        Node(
            package='robot_fsm_controller',
            executable='alignment',
            name='alignment'
        )
        ])
