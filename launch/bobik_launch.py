from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    start_bobik_driver = Node(
        package='bobik_driver',
        executable='bobik_driver',
        name='bobik_driver',
        output='screen',
        arguments=['--honza', 'jenda']
    )

    start_xv_11_driver = Node(
        package='xv_11_driver',
        executable='xv_11_driver',
        name='xv_11_driver',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(start_bobik_driver)
    ld.add_action(start_xv_11_driver)
    return ld
