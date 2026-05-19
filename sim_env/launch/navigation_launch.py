import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    sim_env_dir = get_package_share_directory('sim_env')

    map_name = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    controller_params_file = LaunchConfiguration('controller_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')

    default_params_file = os.path.join(sim_env_dir, 'config', 'nav2_params.yaml')
    default_controller_params_file = os.path.join(
        sim_env_dir, 'config', 'controller_params.yaml'
    )

    param_substitutions = {'use_sim_time': use_sim_time}
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    lifecycle_nodes = [
        'map_server',
        'amcl',
        'planner_server',
        'controller_server',
        'bt_navigator',
        'behavior_server',
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(sim_env_dir, 'maps', 'workshop', 'workshop.yaml'),
        ),
        DeclareLaunchArgument('params_file', default_value=default_params_file),
        DeclareLaunchArgument(
            'controller_params_file',
            default_value=default_controller_params_file,
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                configured_params,
                {'yaml_filename': map_name},
            ],
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                configured_params,
                {
                    'initial_pose_x': x_pose,
                    'initial_pose_y': y_pose,
                    'initial_pose_a': yaw,
                },
            ],
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[
                configured_params,
                controller_params_file,
            ],
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[
                configured_params,
                {
                    'behavior_plugins': ['spin', 'backup', 'wait'],
                    'spin': {'plugin': 'nav2_behaviors/Spin'},
                    'backup': {'plugin': 'nav2_behaviors/BackUp'},
                    'wait': {'plugin': 'nav2_behaviors/Wait'},
                },
            ],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': lifecycle_nodes,
            }],
        ),
    ])
