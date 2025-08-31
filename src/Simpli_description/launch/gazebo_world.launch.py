import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # Process xacro to get robot description
    pkg_path = os.path.join(get_package_share_directory('new_rover_9'))
    xacro_file = os.path.join(pkg_path, 'urdf', 'main_file.xacro')
    robot_description = Command([
        'xacro ', xacro_file,
        ' use_ros2_control:=', use_ros2_control,
        ' sim_mode:=', use_sim_time
    ])

    # Robot state publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'pause': 'true',
            'physics': 'ode',
            'max_step_size': '0.001',
            'real_time_update_rate': '1000',
            'use_sim_time': 'true',
            'verbose': 'true',
            'world': os.path.join(
                get_package_share_directory('leo_erc_gazebo_worlds'),
                'worlds', 'marsyard2020.world.xacro')
        }.items()
    )

    # Spawn the robot into Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_bot',
            '-x', '0', '-y', '0', '-z', '3'
        ],
        output='screen'
    )

    # Spawn joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # Spawn ackermann_steering_controller
    ackermann_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ackermann_steering_controller"],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_ros2_control', default_value='true'),
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        joint_state_broadcaster_spawner,
        ackermann_controller_spawner,
    ])

