#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from pymoveit2 import MoveIt2
from pymoveit2.pymoveit2.robots import kinematic_state
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np
from tf2_ros import TransformListener, Buffer

class ArucoTracking(Node):
    def __init__(self):
        super().__init__('aruco_tracking')
        
        # Initialize logger
        self.logger = get_logger("pymoveit2.aruco_tracking")
        
        # Initialize MoveIt 2
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=["Joint_1,Joint_2,Joint_3,Joint_4,Joint_5,Joint_6,Joint_7"],  # Replace with your robot's joint names
            base_link="world",
            end_effector_link="Link_7",  # Replace with your end effector link
            group_name="hehehe"
        )
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Subscribe to RealSense camera feed
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.camera_callback,
            10
        )
        
        # ArUco dictionary setup
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # TF2 listener for camera to base transform
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Camera intrinsic parameters (update these with your RealSense parameters)
        self.camera_matrix = np.array([[615.0, 0.0, 320.0],
                                     [0.0, 615.0, 240.0],
                                     [0.0, 0.0, 1.0]])
        self.dist_coeffs = np.zeros((4,1))

    async def move_to_pose(self, pose_goal):
        """Helper function to plan and execute a motion."""
        try:
            # Plan and execute motion
            self.logger.info("Planning and executing motion")
            result = await self.moveit2.move_to_pose(pose_goal, auto_execute=True)
            
            if result:
                self.logger.info("Successfully executed motion")
                return True
            else:
                self.logger.error("Failed to execute motion")
                return False
                
        except Exception as e:
            self.logger.error(f"Movement failed: {str(e)}")
            return False

    def camera_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Detect ArUco markers
            corners, ids, _ = cv2.aruco.detectMarkers(
                cv_image,
                self.aruco_dict,
                parameters=self.aruco_params
            )
            
            if ids is not None:
                # Estimate pose of the marker
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners,
                    0.05,  # Marker size in meters
                    self.camera_matrix,
                    self.dist_coeffs
                )
                
                # Get marker position in camera frame
                marker_position = tvecs[0][0]
                
                # Transform marker position to robot base frame
                camera_to_base = self.tf_buffer.lookup_transform(
                    'world',
                    'camera_link',
                    rclpy.time.Time()
                )
                
                # Create PoseStamped message for the target
                pose_goal = PoseStamped()
                pose_goal.header.frame_id = "world"
                pose_goal.pose.position.x = marker_position[0]
                pose_goal.pose.position.y = marker_position[1]
                pose_goal.pose.position.z = marker_position[2]
                pose_goal.pose.orientation.w = 1.0
                
                # Plan and execute
                rclpy.spin_until_future_complete(
                    self,
                    self.move_to_pose(pose_goal)
                )

        except Exception as e:
            self.logger.error(f'Error processing image: {str(e)}')

def main():
    rclpy.init()
    node = ArucoTracking()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()