import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Scene config setup
    scene_config = LaunchConfiguration('scene_config')
    scene_config_arg = DeclareLaunchArgument(
        'scene_config',
        default_value='example_scene1',
        description='Name of scene configuration file'
    )
    
    def get_scene_config_file(context):
        scene_config_value = context.launch_configurations['scene_config']
        return os.path.join(
            get_package_share_directory('constrained_manipulability'),
            'config',
            f'{scene_config_value}.yaml'
        )

    # Declare launch arguments for node parameters
    root_arg = DeclareLaunchArgument(
        'root',
        default_value='base_link',
        description='Base frame of the robot'
    )
    tip_arg = DeclareLaunchArgument(
        'tip',
        default_value='ee_link',
        description='End-effector link of the robot'
    )
    show_mp_arg = DeclareLaunchArgument(
        'show_mp',
        default_value='True',
        description='Whether to show the allowable manipulability polytope or not'
    )
    show_cmp_arg = DeclareLaunchArgument(
        'show_cmp',
        default_value='True',
        description='Whether to show the constrained manipulability polytope or not'
    )
    # Launch configurations
    root = LaunchConfiguration('root')
    tip = LaunchConfiguration('tip')
    show_mp = LaunchConfiguration('show_mp')
    show_cmp = LaunchConfiguration('show_cmp')

    # Define node parameters
    constrained_manip_params = {
        'root': root,
        'tip': tip,
        'show_mp': ParameterValue(show_mp, value_type=bool),
        'show_cmp': ParameterValue(show_cmp, value_type=bool)
    }
    constrained_manip_node = Node(
        package='constrained_manipulability',
        executable='constrained_manipulability_node',
        name='constrained_manipulability_node',
        parameters=[constrained_manip_params]
    )

    return LaunchDescription([
        scene_config_arg,
        root_arg,
        tip_arg,
        show_mp_arg,
        show_cmp_arg,
        constrained_manip_node, 
        OpaqueFunction(function=lambda context: [
            Node(
                package='constrained_manipulability',
                executable='abstract_scene_example',
                name='abstract_scene_example',
                parameters=[get_scene_config_file(context)]
            )
        ])
    ])