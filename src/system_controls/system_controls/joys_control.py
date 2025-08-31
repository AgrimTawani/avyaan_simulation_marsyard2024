import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoystickControl(Node):
    def __init__(self):
        super().__init__('joystick_control')

        # Publisher to the ackermann controller topic
        self.pub = self.create_publisher(
            Twist, 
            '/ackermann_steering_controller/reference_unstamped', 
            10
        )

        # Subscriber to joystick
        self.sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

    def joy_callback(self, msg: Joy):
        twist = Twist()

        # Axis 7 controls forward/backward
        twist.linear.x = msg.axes[7] * 7.0   # scale speed as needed

        # Axis 6 controls left/right turning
        twist.angular.z = msg.axes[6] * 5.0  # adjust steering sensitivity

        self.pub.publish(twist)
        self.get_logger().info(
            f"Publishing: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = JoystickControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
