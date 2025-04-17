from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_cde2310',
            executable='raspi_fire',
            name='raspi_fire'
        ),
        Node(
            package='turtlebot3_cde2310',
            executable='raspi_thermal',
            name='raspi_thermal'
        )
        ])

