import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time

class AMRNode(Node):
    def __init__(self):
        super().__init__('amr_node')
        self.create_subscription(PoseStamped, '/amr/goal_pose', self.goal_callback, 10)
        self.status_pub = self.create_publisher(String, '/amr/status', 10)
        self.ready_pub = self.create_publisher(String, '/system/ready', 10)
        self.ready_pub.publish(String(data="amr_ready"))
        self.get_logger().info("✅ AMR node started and ready.")
        self.ready_pub = self.create_publisher(String, '/system/ready', 10)

        # Timer to publish "amr_ready" every second
        self.create_timer(0.5, self.publish_readiness)

    def publish_readiness(self):
        msg = String()
        msg.data = "amr_ready"
        self.ready_pub.publish(msg)

    def goal_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y

        self.get_logger().info(f"📡 Received goal: Move to x={x:.1f}, y={y:.1f}")
        if abs(y - 2.0) < 0.01:
            self.get_logger().info("🚗 Heading to pallet location...")
            time.sleep(3)  # Simulate moving to pallet
            self.get_logger().info("✅ Arrived at pallet")
            self.status_pub.publish(String(data="arrived_at_pallet"))
        else:
            self.get_logger().info("🚗 Heading to manipulator...")
            time.sleep(2.5)  # Simulate moving to manipulator
            self.get_logger().info("📍 Arrived at manipulator station")
            self.status_pub.publish(String(data="arrived_at_manipulator"))

def main(args=None):
    rclpy.init(args=args)
    node = AMRNode()
    rclpy.spin(node)
    rclpy.shutdown()
