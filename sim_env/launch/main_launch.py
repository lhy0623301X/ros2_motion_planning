import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
)
from launch_ros.actions import Node


def generate_launch_description():
    sim_env_dir = get_package_share_directory('sim_env')
    launch_dir = os.path.join(sim_env_dir, 'launch')

    world = LaunchConfiguration('world')
    map_name = LaunchConfiguration('map')
    robot_type = LaunchConfiguration('robot_type')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    params_file = LaunchConfiguration('params_file')
    controller_params_file = LaunchConfiguration('controller_params_file')
    nav2_start_delay = LaunchConfiguration('nav2_start_delay')

    gazebo_model_paths = os.path.join(sim_env_dir, 'models')

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=gazebo_model_paths,
        ),

        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(sim_env_dir, 'worlds', 'workshop.world'),
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(sim_env_dir, 'maps', 'workshop', 'workshop.yaml'),
        ),
        DeclareLaunchArgument('robot_type', default_value='turtlebot3_waffle'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('nav2_start_delay', default_value='5.0'),
        DeclareLaunchArgument(
            'params_file',
            default_value="/home/lhy/projects_files/ros2_motion_planning/src/sim_env/config/nav2_params.yaml",
        ),
        DeclareLaunchArgument(
            'controller_params_file',
            default_value="/home/lhy/projects_files/ros2_motion_planning/src/sim_env/config/controller_params.yaml",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'gazebo_launch.py')
            ),
            launch_arguments={
                'world': world,
                'use_sim_time': use_sim_time,
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'spawn_robot_launch.py')
            ),
            launch_arguments={
                'robot_type': robot_type,
                'x_pose': x_pose,
                'y_pose': y_pose,
                'yaw': yaw,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        TimerAction(
            period=nav2_start_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, 'navigation_launch.py')
                    ),
                    launch_arguments={
                        'map': map_name,
                        'params_file': params_file,
                        'controller_params_file': controller_params_file,
                        'x_pose': x_pose,
                        'y_pose': y_pose,
                        'yaw': yaw,
                        'use_sim_time': use_sim_time,
                    }.items(),
                ),
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                '-d', os.path.join(sim_env_dir, 'config', 'rviz2_nav2.rviz'),
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(rviz),
        ),
    ])
