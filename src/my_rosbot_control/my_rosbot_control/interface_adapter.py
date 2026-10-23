"""Expose the project-wide robot interface over the official ROSbot API."""

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan


class InterfaceAdapter(Node):
    """Convert velocity commands and publish stable aliases for sensor topics."""

    def __init__(self):
        super().__init__("interface_adapter")

        self.declare_parameter("cmd_vel_input", "/cmd_vel")
        self.declare_parameter("cmd_vel_output", "/rosbot/manual/cmd_vel")
        self.declare_parameter("cmd_vel_frame_id", "rosbot/base_link")
        self.declare_parameter("odom_input", "/rosbot/odometry/filtered")
        self.declare_parameter("odom_output", "/odom")
        self.declare_parameter("wheel_odom_input", "/rosbot/odometry/wheels")
        self.declare_parameter("wheel_odom_output", "/wheel/odom")
        self.declare_parameter("imu_input", "/rosbot/imu/data")
        self.declare_parameter("imu_output", "/imu")
        self.declare_parameter("scan_input", "/rosbot/scan")
        self.declare_parameter("scan_output", "/scan")

        topic = self._string_parameter
        self._cmd_publisher = self.create_publisher(
            TwistStamped, topic("cmd_vel_output"), 10
        )
        self.create_subscription(Twist, topic("cmd_vel_input"), self._cmd_cb, 10)

        self._odom_publisher = self.create_publisher(
            Odometry, topic("odom_output"), 10
        )
        self.create_subscription(
            Odometry, topic("odom_input"), self._odom_publisher.publish, 10
        )

        self._wheel_odom_publisher = self.create_publisher(
            Odometry, topic("wheel_odom_output"), 10
        )
        self.create_subscription(
            Odometry,
            topic("wheel_odom_input"),
            self._wheel_odom_publisher.publish,
            10,
        )

        self._imu_publisher = self.create_publisher(
            Imu, topic("imu_output"), qos_profile_sensor_data
        )
        self.create_subscription(
            Imu,
            topic("imu_input"),
            self._imu_publisher.publish,
            qos_profile_sensor_data,
        )

        self._scan_publisher = self.create_publisher(
            LaserScan, topic("scan_output"), qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan,
            topic("scan_input"),
            self._scan_publisher.publish,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            "Compatibility interface active: /cmd_vel (Twist), /odom, "
            "/wheel/odom, /imu and /scan"
        )

    def _string_parameter(self, parameter_name):
        return self.get_parameter(parameter_name).get_parameter_value().string_value

    def _cmd_cb(self, message):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self._string_parameter("cmd_vel_frame_id")
        stamped.twist = message
        self._cmd_publisher.publish(stamped)


def main(args=None):
    rclpy.init(args=args)
    node = InterfaceAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
