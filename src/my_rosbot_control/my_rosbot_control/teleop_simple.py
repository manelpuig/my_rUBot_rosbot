import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class SimpleTeleop(Node):

    def __init__(self):
        super().__init__('simple_teleop')

        self.pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        self.timer = self.create_timer(0.1, self.timer_cb)
        self.start_time = self.get_clock().now()

        self.get_logger().info('Simple ROSbot teleop started')

    def timer_cb(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

        if elapsed < 5.0:
            msg.twist.linear.x = 0.3   # m/s
            msg.twist.angular.z = 0.0
        else:
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.0

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SimpleTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
