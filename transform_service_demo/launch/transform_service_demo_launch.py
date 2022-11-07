"""
Some help from: https://medium.com/@danieljeswin/introduction-to-programming-with-ros2-launch-files-52eac873f9d0
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import launch_ros.actions


def generate_launch_description():

    # We need to include an external launch file to start the simulator...
    tb_prefix = get_package_share_directory('turtlebot3_gazebo')
    included_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([tb_prefix,
                                            '/launch/turtlebot3_world.launch.py']))

    # We need to get the path to the .rviz file...
    this_prefix = get_package_share_directory('transform_service_demo')
    rviz_path = os.path.join(this_prefix, 'rviz', 'overhead.rviz')


    return LaunchDescription([
        Node(
            package="transform_service_demo",
            executable="transform_tester",
            name="transform_tester",
            output="screen",
            parameters=[
                {"use_sim_time": True}
            ]
        ),
        Node(
            package="transform_service",
            executable="transform_service",
            name="transform_service",
            output="screen",
            parameters=[
                {"use_sim_time": True}
            ]
        ),
        Node(
            package="rviz2",
            node_executable="rviz2",
            node_name="rviz2",
            output="screen",
            parameters=[
                {"use_sim_time": True}
            ],
            arguments=['-d', rviz_path]
        ),
        included_launch

    ])
if __name__ == "__main__":
    print(generate_launch_description().entities)
