from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bobik_driver',
            namespace='bobik_driver',
            executable='bobik_driver',
            name='bobik_driver'
        ),
        Node(
            package='xv_11_driver',
            namespace='xv_11_driver',
            executable='xv_11_driver',
            name='lidar'
        )
    ])
