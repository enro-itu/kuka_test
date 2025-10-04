from launch import LaunchDescription
from launch_ros.actions import Node
import subprocess

def generate_launch_description():
    urdf_file = "/home/yagizcumen/rk_ws/src/rk_demo/urdf/robot_arm.urdf.xacro"

    # xacro'yu Python'dan çalıştır ve çıktıyı al
    robot_description_config = subprocess.check_output(
        ["/opt/ros/jazzy/bin/xacro", urdf_file]
    ).decode('utf-8')

    return LaunchDescription([
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen"
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description_config
            }]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", "/home/yagizcumen/rk_ws/src/rk_demo/config/view.rviz"],
            output="screen"
)

    ])
