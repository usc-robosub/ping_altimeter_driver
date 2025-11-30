import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Range
from geometry_msgs.msg import Pose
from brping import Ping1D


class Ping1DAltimeterNode(Node):
    def __init__(self):
        # Node name matches ROS1 init_node name
        super().__init__('ping1d_altimeter_node')

        # Declare parameters with SAME NAMES & DEFAULTS as ROS1
        self.declare_parameter('port', '/dev/serial0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'ping1d')
        self.declare_parameter('rate', 10.0)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.rate = self.get_parameter('rate').get_parameter_value().double_value

        # Initialize Ping1D device
        self.ping = Ping1D()
        self.get_logger().info(f"Connecting to Ping1D on {self.port} at {self.baudrate} bps...")
        self.ping.connect_serial(self.port, self.baudrate)

        if not self.ping.initialize():
            self.get_logger().error("Failed to initialize Ping1D!")
            raise RuntimeError("Ping1D device not responding.")

        self.get_logger().info("Ping1D initialized successfully.")

        # Publishers (same names & types as ROS1)
        self.range_pub = self.create_publisher(Range, 'ping1d/range', 10)
        self.pose_pub = self.create_publisher(Pose, 'ping1d/pose', 10)

        # Timer to mimic rospy.Rate(self.rate)
        period = 1.0 / self.rate if self.rate > 0.0 else 0.1
        self.timer = self.create_timer(period, self.timer_callback)

    def timer_callback(self):
        data = self.ping.get_distance()
        if not data:
            self.get_logger().warn("Failed to read distance from Ping1D.")
            return

        distance_m = data['distance'] / 1000.0  # mm -> m

        # Range message
        range_msg = Range()
        range_msg.header.stamp = self.get_clock().now().to_msg()
        range_msg.header.frame_id = self.frame_id
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = 0.1
        range_msg.min_range = 0.02
        range_msg.max_range = 30.0
        range_msg.range = distance_m

        # Pose message (no header in Pose)
        pose_msg = Pose()
        pose_msg.position.x = 0.0
        pose_msg.position.y = 0.0
        pose_msg.position.z = distance_m

        self.range_pub.publish(range_msg)
        self.pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = Ping1DAltimeterNode()
    except Exception as e:
        print(f"Ping1DAltimeterNode failed to start: {e}")
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
