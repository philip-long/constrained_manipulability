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
            '/abstract_robot.launch.py']),
        launch_arguments={
            'scene_config': 'example_scene_gen3',
            'root': 'base_link',
            'tip': 'bracelet_link',
            'show_mp': 'True',
            'show_cmp': 'True'}.items()
    )
    
    # Kinova Gen3 launch
    # Beware that if using the ROS 2 Humble distro release, a joint_state_publisher
    # is in `view_robot.launch.py` and it will conflict with the constrained_manipulability
    # method of updating the joint states via the GUI. See Git Issue #10 for more info.
    gen3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('kortex_description'), 'launch'),
            '/view_robot.launch.py']),
        launch_arguments={
            'robot_type': 'gen3',
            'dof': '7'}.items()
    )

    return LaunchDescription([
        abstract_robot_launch,
        gen3_launch
    ])