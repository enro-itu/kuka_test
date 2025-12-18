from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_path = get_package_share_directory('rk_demo')

    controllers_yaml = os.path.join(pkg_path, 'config', 'controllers.yaml')

    return LaunchDescription([

        Node(
            package='rk_demo',
            executable='arm_ui',
            name='rk_arm_ui',
            output='screen'
        ),

       
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                controllers_yaml,   
            ],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_group_position_controller'],
            output='screen'
        ),
    ])
