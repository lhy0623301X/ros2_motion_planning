from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_type = LaunchConfiguration('robot_type')
    robot_name = LaunchConfiguration('robot_name')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw = LaunchConfiguration('yaw')
    use_sim_time = LaunchConfiguration('use_sim_time')

    xacro_path = [
        FindPackageShare('sim_env'), '/urdf/', robot_type, '/', robot_type, '.xacro'
    ]

    robot_description = Command(['xacro', ' ', *xacro_path])

    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value='turtlebot3_waffle'),
        DeclareLaunchArgument('robot_name', default_value='turtlebot3_waffle'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('z_pose', default_value='0.01'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
            }],
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-entity', robot_name,
                '-x', x_pose,
                '-y', y_pose,
                '-z', z_pose,
                '-Y', yaw,
            ],
        ),
    ])
