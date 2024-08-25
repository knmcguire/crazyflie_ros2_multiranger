import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_crazyflie_gazebo = get_package_share_directory('ros_gz_crazyflie_bringup')

    # Setup to launch a crazyflie gazebo simulation from the ros_gz_crazyflie project
    crazyflie_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_crazyflie_gazebo, 'launch', 'crazyflie_simulation.launch.py'))
    )

    # start a simple mapper node
    simple_mapper = Node(
        package='crazyflie_ros2_multiranger_simple_mapper',
        executable='simple_mapper_multiranger',
        name='simple_mapper',
        output='screen',
    )

    return LaunchDescription([
        crazyflie_simulation,
        simple_mapper
        ])