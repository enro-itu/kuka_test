from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = get_package_share_directory('rk_demo')
    urdf_path = os.path.join(pkg_path, 'urdf', 'robot_arm.urdf.xacro')

    return LaunchDescription([

        # Gazebo başlat
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch', 'gz_sim.launch.py')]
            ),
            launch_arguments={
                'gz_args': '-r empty.sdf'
            }.items()
        ),

        # Robot State Publisher (robot_description paramı üretir)
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": Command([
                    "xacro ", urdf_path
                ])
            }]
        ),

        # Gazebo'ya spawn et
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'my_robot',
                '-topic', 'robot_description'
            ],
            output='screen'
        ),
    ])
