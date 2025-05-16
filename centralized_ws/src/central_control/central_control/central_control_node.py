import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import threading
from std_msgs.msg import Int32


class CentralControl(Node):
    def __init__(self):
        super().__init__('central_control_node')
        self.pallet_id = 0
        self.state = "idle"
        self.amr_ready = False
        self.manip_ready = False
        self.system_ready = False
        self.waiting_for_task = False


        # Subscriptions
        self.create_subscription(String, '/system/ready', self.ready_callback, 10)
        self.create_subscription(String, '/amr/status', self.amr_status_callback, 10)
        self.create_subscription(String, '/manipulator/status', self.manip_status_callback, 10)
        self.create_subscription(Int32, '/pallet_request', self.pallet_callback, 10)


        # Publishers
        self.amr_goal_pub = self.create_publisher(PoseStamped, '/amr/goal_pose', 10)
        self.manip_cmd_pub = self.create_publisher(String, '/manipulator/command', 10)
        self.state_pub = self.create_publisher(String, '/system/state', 10)

        # Start input thread (only active when system_ready is True)
        thread = threading.Thread(target=self.input_loop)
        thread.daemon = True
        thread.start()

    def ready_callback(self, msg):
        if msg.data == "amr_ready":
            self.amr_ready = True
        elif msg.data == "manip_ready":
            self.manip_ready = True

        if self.amr_ready and self.manip_ready:
            self.system_ready = True  # Set the flag

    def input_loop(self):
        while not self.system_ready:
            pass

        self.get_logger().info("✅ All systems are ready. Waiting for pallet input...")

        while rclpy.ok():
            if self.waiting_for_task:
                continue  # Wait for current task to finish

            try:
                pallet_number = int(input("\n📥 Enter pallet number (1-10): "))
                if 1 <= pallet_number <= 10:
                    self.pallet_id = pallet_number
                    self.waiting_for_task = True  # Disable input until done
                    self.send_amr_to_pallet()
                else:
                    print("❌ Please enter a number between 1 and 10.")
            except ValueError:
                print("❌ Invalid input. Enter an integer between 1 and 10.")

    def send_amr_to_pallet(self):
        pose = PoseStamped()
        pose.header.frame_id = "map"

        pallet_locations = {
            1: (1.0, 2.0), 2: (1.2, 2.0), 3: (1.4, 2.0),
            4: (1.6, 2.0), 5: (1.8, 2.0), 6: (2.0, 2.0),
            7: (2.2, 2.0), 8: (2.4, 2.0), 9: (2.6, 2.0), 10: (2.8, 2.0)
        }

        x, y = pallet_locations[self.pallet_id]
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0

        self.amr_goal_pub.publish(pose)
        self.state = "amr_to_pallet"
        self.report_state(f"🚗 AMR moving to pallet {self.pallet_id} at ({x}, {y})")

    def send_amr_to_manipulator(self):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = 3.0
        pose.pose.position.y = 1.0
        pose.pose.orientation.w = 1.0

        self.amr_goal_pub.publish(pose)
        self.state = "amr_to_manipulator"
        self.report_state("🚗 AMR moving to manipulator drop zone")

    def amr_status_callback(self, msg):
        if msg.data == "arrived_at_pallet" and self.state == "amr_to_pallet":
            self.report_state(f"✅ AMR picked up pallet {self.pallet_id}")
            self.send_amr_to_manipulator()

        elif msg.data == "arrived_at_manipulator" and self.state == "amr_to_manipulator":
            self.report_state("📍 AMR arrived at manipulator station")
            self.manip_cmd_pub.publish(String(data="pick_basket"))
            self.state = "manipulator_picking"
            self.report_state("🦾 Manipulator instructed to pick basket")

    def manip_status_callback(self, msg):
        if msg.data == "picked" and self.state == "manipulator_picking":
            self.report_state("✅ Manipulator picked the basket. Task complete!")
            self.state = "idle"
            self.waiting_for_task = False  # Enable next input

    def pallet_callback(self, msg):
        if not self.system_ready:
            self.get_logger().warn("System not ready yet.")
            return

        if self.waiting_for_task:
            self.get_logger().warn("Still processing previous task. Wait...")
            return

        pallet_number = msg.data
        if 1 <= pallet_number <= 10:
            self.pallet_id = pallet_number
            self.waiting_for_task = True
            self.send_amr_to_pallet()
        else:
            self.get_logger().warn("Invalid pallet number received. Must be 1-10.")

    def report_state(self, msg):
        self.get_logger().info(msg)
        self.state_pub.publish(String(data=msg))

def main(args=None):
    rclpy.init(args=args)
    node = CentralControl()
    rclpy.spin(node)
    rclpy.shutdown()
