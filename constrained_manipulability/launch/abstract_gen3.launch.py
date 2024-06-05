import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Abstract robot launch
    abstract_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('constrained_manipulability'), 'launch'),
            '/abstract_scene_example.launch.py']),
        launch_arguments={
            'scene_config': 'example_scene_gen3',
            'root': 'base_link',
            'tip': 'bracelet_link',
            'show_mp': 'True',
            'show_cmp': 'True'}.items()
    )
    
    # UR launch
    ur_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('kortex_description'), 'launch'),
            '/view_robot.launch.py']),
        launch_arguments={
            'robot_type': 'gen3',
            'dof': '7'}.items()
    )

    return LaunchDescription([
        abstract_robot_launch,
        ur_launch
    ])