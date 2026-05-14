from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = '/home/lhy/projects_files/ros2_motion_planning/src/sim_env/worlds/workshop.world'

    return LaunchDescription([
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
                'verbose': 'false',
                'world': world,
            }.items(),
        ),
    ])
