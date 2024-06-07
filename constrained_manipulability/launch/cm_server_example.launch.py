from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Run CM node which contains the polytope server (assumes UR3e robot)
    constrained_manip_node = Node(
        package='constrained_manipulability',
        executable='constrained_manipulability_node',
        name='constrained_manipulability_node',
        parameters=[{'root': 'base_link',
                     'tip': 'wrist_3_link',
                     'show_mp': False,
                     'show_cmp': False,
                     'filter_robot': False}]
    )
    
    # Load the UR3e robot description and state publisher
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([FindPackageShare('ur_description'), 'urdf', 'ur.urdf.xacro']),
            ' ',
            'safety_limits:=true ',
            'safety_pos_margin:=0.15 ',
            'safety_k_position:=20 ',
            'name:=ur ',
            'ur_type:=ur3e ',
            'tf_prefix:=""'
        ]
    )
    robot_description = {'robot_description': robot_description_content}
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )
    
    # Octomap server setup
    create_cloud_xyzrgb = Node(
        package='octomap_filter',
        executable='create_cloud_xyzrgb.py',
        name='create_cloud_xyzrgb'
    )

    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        parameters=[
            {'resolution': 0.05},
            {'frame_id': 'map'},
            {'sensor_model/max_range': 4.0}
        ],
        remappings=[
            ('cloud_in', 'artificial_cloud')
        ]
    )
    
    octomap_filter_node = Node(
        package='octomap_filter',
        executable='octomap_filter_node',
        name='octomap_filter',
        output='screen'
    )
        
    map_to_world_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_world_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'world']
    )
        
    world_to_base_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link']
    )
        
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', str(get_package_share_directory('constrained_manipulability_interfaces')) + '/rviz/octomap_robot.rviz']
    )

    return LaunchDescription([
        constrained_manip_node,
        robot_state_publisher_node,
        create_cloud_xyzrgb,
        octomap_server_node,
        octomap_filter_node,
        map_to_world_broadcaster,
        world_to_base_broadcaster,
        rviz_node
    ])