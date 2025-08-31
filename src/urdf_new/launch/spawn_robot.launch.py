# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, ExecuteProcess
# from launch.launch_description_sources import PythonLaunchDescriptionSource


# from launch_ros.actions import Node
# import xacro


# def generate_launch_description():
    
#     # Specify the name of the package and path to xacro file within the package
#     pkg_name = 'main_pan_aruco'
#     file_subpath = 'config/urdf_new.urdf.xacro'

#     pkg_name_1 = 'sulekha'
#     file_subpath_1 = 'urdf/main_pan_aruco.urdf'

#     # Use xacro to process the file
#     xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
#     robot_description_raw = xacro.process_file(xacro_file).toxml()


#     # Configure the node
#     node_robot_state_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[{'robot_description': robot_description_raw,
#         'use_sim_time': True}] # add other parameters here if required
#     )



#     # gazebo = IncludeLaunchDescription(
#     #     PythonLaunchDescriptionSource([os.path.join(
#     #         get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
#     #     )
#     gazebo = IncludeLaunchDescription(
#     PythonLaunchDescriptionSource([os.path.join(
#         get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
#     launch_arguments={
#         'pause': 'false',
#         'physics': 'ode',
#         'max_step_size': '0.001',
#         'real_time_update_rate': '1000',
#         'use_sim_time': 'true'
#     }.items()
# )


#     spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
#                     arguments=['-topic', 'robot_description',
#                                 '-entity', 'my_bot',],
#                     output='screen')
    
    

#     # start_arm_controller_cmd = ExecuteProcess(
#     # cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
#     #     ''],
#     #     output='screen')

#     # Run the node
#     return LaunchDescription([
#         gazebo,
#         node_robot_state_publisher,
#         spawn_entity
#     ])


# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource

# from launch_ros.actions import Node
# import xacro


# def generate_launch_description():
    
#     # Load Maintenance Panel URDF
#     pkg_name_panel = 'main_pan_aruco'
#     file_subpath_panel = 'urdf/main_pan_aruco.urdf' 

#     xacro_file_panel = os.path.join(get_package_share_directory(pkg_name_panel), file_subpath_panel)
#     robot_description_panel = xacro.process_file(xacro_file_panel).toxml()

#     # Load Robotic Arm URDF
#     pkg_name_arm = 'sulekha'
#     file_subpath_arm = 'config/urdf_new.urdf.xacro'

#     xacro_file_arm = os.path.join(get_package_share_directory(pkg_name_arm), file_subpath_arm)
#     robot_description_arm = xacro.process_file(xacro_file_arm).toxml()

#     # Start Robot State Publisher for Maintenance Panel
#     node_robot_state_publisher_panel = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[{'robot_description': robot_description_panel,
#                      'use_sim_time': True}],
#         remappings=[('/robot_description', '/maintenance_panel_description')]
#     )

#     # Start Robot State Publisher for Robotic Arm
#     node_robot_state_publisher_arm = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[{'robot_description': robot_description_arm,
#                      'use_sim_time': True}],
#         remappings=[('/robot_description', '/robotic_arm_description')]
#     )

#     # Launch Gazebo
#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([os.path.join(
#             get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
#         launch_arguments={
#             'pause': 'false',
#             'physics': 'ode',
#             'max_step_size': '0.001',
#             'real_time_update_rate': '1000',
#             'use_sim_time': 'true'
#         }.items()
#     )

#     # Spawn Maintenance Panel
#     spawn_panel = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         arguments=['-topic', '/maintenance_panel_description',
#                    '-entity', 'maintenance_panel'],
#         output='screen'
#     )

#     # Spawn Robotic Arm
#     spawn_arm = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         arguments=['-topic', '/robotic_arm_description',
#                    '-entity', 'robotic_arm'],
#         output='screen'
#     )

#     return LaunchDescription([
#         gazebo,
#         node_robot_state_publisher_panel,
#         node_robot_state_publisher_arm,
#         spawn_panel,
#         spawn_arm
#     ])


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import time

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    launch_file_dir = os.path.join(get_package_share_directory('gazebo_simulation'), 'launch')
    # Load Maintenance Panel URDF (Read file and remove XML declaration if present)
    pkg_name_panel = 'main_pan_rev'
    file_subpath_panel = 'urdf/main_pan_rev.urdf' 
    # pkg_name_panel = 'new_rover_9'
    # file_subpath_panel = 'urdf/new_rover_9.urdf'

    urdf_file_panel = os.path.join(get_package_share_directory(pkg_name_panel), file_subpath_panel)
    with open(urdf_file_panel, 'r', encoding='utf-8') as infp:
        robot_description_panel = infp.read().replace('<?xml version="1.0" encoding="utf-8"?>', '')

    # Load Robotic Arm URDF (Xacro file needs processing)
    pkg_name_arm = '3sulekha'
    file_subpath_arm = 'config/shaunak_ra.urdf.xacro'

    # pkg_name_arm = 'new_rover_9'
    # file_subpath_arm = 'urdf/main_file.xacro'

    xacro_file_arm = os.path.join(get_package_share_directory(pkg_name_arm), file_subpath_arm)
    robot_description_arm = xacro.process_file(xacro_file_arm).toxml().replace('<?xml version="1.0" encoding="utf-8"?>', '')
    pkg_four_ws_control = get_package_share_directory('four_ws_control')
    # Start Robot State Publisher for Maintenance Panel
    node_robot_state_publisher_panel = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_panel,
                     'use_sim_time': True}],
        remappings=[('/robot_description', '/maintenance_panel_description')]
    )
    
    # Start Robot State Publisher for Robotic Arm
    node_robot_state_publisher_arm = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_arm,
                     'use_sim_time': True}],
        remappings=[('/robot_description', '/robotic_arm_description')]
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
            # 'world': os.path.join(get_package_share_directory('leo_erc_gazebo_worlds'), 'worlds', 'marsyard2020.world.xacro')
        }.items()
    )
    # time.sleep(1)
    # Spawn Maintenance Panel at (0, 1, 0)
    spawn_panel = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/maintenance_panel_description',
                   '-entity', 'maintenance_panel',
                   '-x', '-1', '-y', '0', '-z', '0'],
        output='screen'
    )
    time.sleep(1)
    # Spawn Robotic Arm at (0, 0, 0)
    spawn_arm = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robotic_arm_description',
                   '-entity', 'robotic_arm',
                   '-x', '0', '-y', '0', '-z', '0'],
        output='screen'
    )

    time.sleep(1)
    controller = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_four_ws_control, 'launch', 'four_ws_control.launch.py')
            ),
        )
    
    robot_state_publisher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )

    forward_position_controller = ExecuteProcess( 
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'bahaha_controller'], output='screen'
        )

    forward_velocity_controller = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'forward_velocity_controller'], output='screen'
        )

    joint_state_broadcaster = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'], output='screen'
        )
        
    joy_node = Node(
        package = "joy",
        executable = "joy_node"
        )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher_panel,
        node_robot_state_publisher_arm,
        spawn_panel,
        spawn_arm,
        # controller,
        forward_position_controller,
        # forward_velocity_controller,
        joint_state_broadcaster,
        joy_node

    ])


