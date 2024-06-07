import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Abstract robot launch
    abstract_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('constrained_manipulability'), 'launch'),
            '/abstract_robot.launch.py']),
        launch_arguments={
            'scene_config': 'example_scene_ur3',
            'root': 'base_link',
            'tip': 'wrist_3_link',
            'show_mp': 'False',
            'show_cmp': 'True'}.items()
    )
    
    # UR launch
    ur_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ur_description'), 'launch'),
            '/view_ur.launch.py']),
        launch_arguments={'ur_type': 'ur3e'}.items()
    )
    
    lin_limit_pub = Node(
        package='constrained_manipulability',
        executable='lin_limit_pub.py',
        name='lin_limit_pub',
    )

    return LaunchDescription([
        abstract_robot_launch,
        ur_launch,
        lin_limit_pub
    ])