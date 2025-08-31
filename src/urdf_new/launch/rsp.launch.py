import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,ExecuteProcess
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import time

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # launch_file_dir = os.path.join(get_package_share_directory('gazebo_simulation'), 'launch')
    pkg_name_arm = 'new_rover_9'
    file_subpath_arm = 'urdf/main_file.xacro'

    xacro_file_arm = os.path.join(get_package_share_directory(pkg_name_arm), file_subpath_arm)
    robot_description_arm = xacro.process_file(xacro_file_arm).toxml().replace('<?xml version="1.0" encoding="utf-8"?>', '')

    
    # # Start Robot State Publisher for Robotic Arm
    node_robot_state_publisher_arm = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_arm,
                     'use_sim_time': True}],
        # remappings=[('/robot_description', '/robotic_arm_description')]
    )

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={
            'pause': 'false',
            'physics': 'ode',
            'max_step_size': '0.001',
            'real_time_update_rate': '1000',
            'use_sim_time': 'true',
            'verbose': 'true',
            'world': os.path.join(get_package_share_directory('leo_erc_gazebo_worlds'), 'worlds', 'marsyard2020.world.xacro')
        }.items()
    )

    spawn_arm = TimerAction(
    period=3.0,  
    actions=[Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
                   '-entity', 'robotic_arm',
                   '-x', '0', '-y', '0', '-z', '3'],
        output='screen'
    )]
  ) 
    # pkg_four_ws_control = get_package_share_directory('four_ws_control')
    # controller = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             os.path.join(pkg_four_ws_control, 'launch', 'four_ws_control.launch.py')
    #         ),
    #     )
    
    # robot_state_publisher = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
    #         launch_arguments={'use_sim_time': use_sim_time}.items(),
    #     )

    # forward_position_controller = ExecuteProcess( 
    #         cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'forward_position_controller'], output='screen'
    #     )

    # forward_velocity_controller = ExecuteProcess(
    #         cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'forward_velocity_controller'], output='screen'
    #     )

    # joint_state_broadcaster = ExecuteProcess(
    #         cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'], output='screen'
    # )    

    # joint_state_publisher = Node(
    #             package="joint_state_publisher",
    #             executable="joint_state_publisher",
    #             name="joint_state_publisher",
    #             parameters=[{'use_sim_time': True}],
    #         )

    # joy_node = Node(
    #     package = "joy",
    #     executable = "joy_node"
    #     )
    
    # base_odom = Node(
    #     package='ur5_control',
    #     executable='odmom_base_link',
    #     parameters=[{'use_sim_time': True}],
    #     )
    
    # ekf_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=['/home/agrim/dev_ws/install/urdf_new/share/urdf_new/config/ekf.yaml'],
    #     arguments=['--ros-args', '--param', 'use_sim_time:=true']  # Set use_sim_time to true
    #     )
    
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher_arm,
        spawn_arm
        # forward_position_controller,
        # forward_velocity_controller,
        # joint_state_broadcaster,
        # controller,
        # joy_node,
        # # joint_state_publisher,
        # # ekf_node,
        # base_odom
    ])
