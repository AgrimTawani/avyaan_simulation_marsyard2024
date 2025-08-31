cmake_minimum_required(VERSION 3.8)
project(leo_erc_gazebo_worlds)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(rclpy REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

# install(PROGRAMS
#   src/moveto_aruco.py  # Adjust the path based on where you save the script
#   DESTINATION lib/${PROJECT_NAME}
# )

# # Install Python modules if you have any
# install(DIRECTORY
#   src/  # If you have additional Python modules
#   DESTINATION lib/${PROJECT_NAME}
# )
# Install URDF files
install(DIRECTORY media/
  DESTINATION share/${PROJECT_NAME}/media
)

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# ament package
ament_package()
    