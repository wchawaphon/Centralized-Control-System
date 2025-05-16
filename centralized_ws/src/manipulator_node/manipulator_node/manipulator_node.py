import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class ManipulatorNode(Node):
    def __init__(self):
        super().__init__('manipulator_node')
        self.command_sub = self.create_subscription(String, '/manipulator/command', self.cmd_callback, 10)
        self.status_pub = self.create_publisher(String, '/manipulator/status', 10)
        self.busy = False
        self.ready_pub = self.create_publisher(String, '/system/ready', 10)
        self.ready_pub.publish(String(data="manip_ready"))
        self.get_logger().info("✅ Manipulator node started and ready.")
        self.ready_pub = self.create_publisher(String, '/system/ready', 10)

        # Timer to publish "manip_ready" every second
        self.create_timer(0.5, self.publish_readiness)

    def publish_readiness(self):
        msg = String()
        msg.data = "manip_ready"
        self.ready_pub.publish(msg)

    def cmd_callback(self, msg):
        if self.busy:
            self.get_logger().warn("Manipulator is currently busy.")
            return

        command = msg.data

        if command == "pick_basket":
            self.busy = True
            self.get_logger().info("🦾 Command received: Pick basket")
            self.get_logger().info("🦾 Reaching to basket...")
            time.sleep(2.0)  # Reaching
            self.get_logger().info("🦾 Grasping and lifting basket...")
            time.sleep(1.5)  # Grasp
            self.get_logger().info("✅ Basket picked successfully")
            self.status_pub.publish(String(data="picked"))
            self.busy = False
        else:
            self.get_logger().warn(f"Unknown command received: {command}")

def main(args=None):
    rclpy.init(args=args)
    node = ManipulatorNode()
    rclpy.spin(node)
    rclpy.shutdown()
