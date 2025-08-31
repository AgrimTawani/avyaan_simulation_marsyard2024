# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from ament_index_python.packages import get_package_share_directory
# import os

# def generate_launch_description():
#     gazebo_launch_file = os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
#     urdf_file = os.path.join(get_package_share_directory('new_rover_9'), 'urdf', 'new_rover_9.urdf')

#     return LaunchDescription([
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(gazebo_launch_file)
#         ),
        
#         # Node(
#             package='tf2_ros',
#             executable='static_transform_publisher',
#             name='tf_footprint_base',
#             arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint', '40']
#         ),
        
#         Node(
#             package='gazebo_ros',
#             executable='spawn_entity.py',
#             name='spawn_model',
#             arguments=['-file', urdf_file, '-urdf', '-entity', 'new_rover_9'],
#             output='screen'
#         ),
        
#         Node(
#             package='std_msgs',
#             executable='ros2',
#             name='fake_joint_calibration',
#             arguments=['topic', 'pub', '/calibrated', 'std_msgs/Bool', 'true']
#         )
#     ])
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_name = "new_rover_9"
    rsp=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(pkg_name),'launch','rsp.launch.py'
        )]), launch_arguments={'use_sim_time':'true'}.items()
    )
    gazebo=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py'
        )]),
    )
    spawn_entity=Node(package='gazebo_ros', executable='spawn_entity.py',
                      arguments=['-topic','robot_description',
                                 '-entity','my_bot'],  
                      output='screen')
    
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
    ])