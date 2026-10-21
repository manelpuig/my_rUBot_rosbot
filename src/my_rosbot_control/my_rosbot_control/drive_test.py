"""Publish a bounded, parameterized ROSbot velocity test."""

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node


class DriveTest(Node):
    """Publish configurable planar velocity commands and then stop."""

    def __init__(self):
        super().__init__("drive_test")
        self.declare_parameter("vx", 0.0)
        self.declare_parameter("vy", 0.0)
        self.declare_parameter("wz", 0.0)
        self.declare_parameter("duration", 5.0)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("stop_cycles", 5)

        self.vx = float(self.get_parameter("vx").value)
        self.vy = float(self.get_parameter("vy").value)
        self.wz = float(self.get_parameter("wz").value)
        self.duration = float(self.get_parameter("duration").value)
        publish_rate = float(self.get_parameter("publish_rate").value)
        self.stop_cycles = int(self.get_parameter("stop_cycles").value)

        if self.duration < 0.0:
            raise ValueError("duration must be non-negative")
        if publish_rate <= 0.0:
            raise ValueError("publish_rate must be positive")
        if self.stop_cycles < 1:
            raise ValueError("stop_cycles must be at least 1")

        self.publisher = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self.start_time = self.get_clock().now()
        self.stops_published = 0
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        self.get_logger().info(
            "Velocity test: "
            f"vx={self.vx:.3f} m/s, vy={self.vy:.3f} m/s, "
            f"wz={self.wz:.3f} rad/s, duration={self.duration:.3f} s"
        )

    def timer_callback(self):
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

        if elapsed < self.duration:
            message.twist.linear.x = self.vx
            message.twist.linear.y = self.vy
            message.twist.angular.z = self.wz
        else:
            self.stops_published += 1

        self.publisher.publish(message)

        if self.stops_published >= self.stop_cycles:
            self.get_logger().info("Drive test completed; the robot is stopped")
            self.timer.cancel()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = DriveTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            stop_message = TwistStamped()
            stop_message.header.stamp = node.get_clock().now().to_msg()
            node.publisher.publish(stop_message)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
