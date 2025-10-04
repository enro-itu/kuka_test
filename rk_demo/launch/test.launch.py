from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command([
                    '/opt/ros/jazzy/bin/xacro',
                    '/home/yagizcumen/rk_ws/src/rk_demo/urdf/robot_arm.urdf.xacro'
                ])
            }]
        )
    ])
