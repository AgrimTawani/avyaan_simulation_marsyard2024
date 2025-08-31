from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='',
        description='Robot model file'
    )

    urdf_file = os.path.join(get_package_share_directory('new_rover_9'), 'urdf', 'new_rover_9.urdf')
    rviz_config_file = os.path.join(get_package_share_directory('new_rover_9'), 'urdf.rviz')

    return LaunchDescription([
        model_arg,
        
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': urdf_file}]
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config_file]
        )
    ])