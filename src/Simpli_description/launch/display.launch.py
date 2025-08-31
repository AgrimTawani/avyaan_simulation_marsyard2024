import os
import xacro
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = "Simpli_description"

    xacro_file = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        'Simpli.xacro'
    )

    # Process the Xacro file
    robot_description_config = xacro.process_file(xacro_file).toxml()

    return launch.LaunchDescription([
        # Joint State Publisher
        launch_ros.actions.Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen"
        ),

        # Robot State Publisher (loads URDF from Xacro)
        launch_ros.actions.Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description_config}]
        ),

        # # Joint State Publisher
        # launch_ros.actions.Node(
        #     package="joint_state_publisher",
        #     executable="joint_state_publisher",
        #     name="joint_state_publisher",
        #     output="screen"
        # ),

        # # Robot State Publisher (loads URDF from Xacro)
        # launch_ros.actions.Node(
        #     package="robot_state_publisher",
        #     executable="robot_state_publisher",
        #     name="robot_state_publisher",
        #     output="screen",
        #     parameters=[{"robot_description": Command([xacro_file])}]

        # ),

        # Controller Manager
        launch_ros.actions.Node(
            package="controller_manager",
            executable="ros2_control_node",
            name="controller_manager",
            output="screen",
            parameters=[{"robot_description": robot_description_config}]
        ),

        # Spawn the robot in Gazebo
        launch_ros.actions.Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="spawn_robot",
            arguments=["-topic", "/robot_description", "-entity", "Simpli"],
            output="screen",
        ),

        # RViz
        launch_ros.actions.Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", PathJoinSubstitution([FindPackageShare(package_name), "launch"])],
        ),
    ])
