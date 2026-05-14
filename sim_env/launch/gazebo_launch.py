import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_env_share = get_package_share_directory('sim_env')

    world = LaunchConfiguration(
        'world',
        default=PathJoinSubstitution(
            [
                FindPackageShare('sim_env'),
                'worlds',
                'workshop.world',
            ]
        )
    )
    model_path = os.path.join(sim_env_share, 'models')
    resource_paths = [
        sim_env_share,
        model_path,
        '/usr/share/gazebo-11',
        '/usr/share/gazebo-11/media',
    ]

    model_paths = [model_path]

    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH')
    if existing_model_path:
        model_paths.append(existing_model_path)

    existing_resource_path = os.environ.get('GAZEBO_RESOURCE_PATH')
    if existing_resource_path:
        resource_paths.append(existing_resource_path)

    gazebo_model_path = os.pathsep.join(model_paths)
    gazebo_resource_path = os.pathsep.join(resource_paths)

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=world,
            description='Directory of gazebo world file',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', gazebo_resource_path),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare('gazebo_ros'),
                            'launch',
                            'gazebo.launch.py',
                        ]
                    )
                ]
            ),
            launch_arguments={
                'world': world,
                'verbose': 'true',
            }.items(),
        ),
    ])
