import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import threading
import time

app = FastAPI()

# Allow frontend JavaScript to connect
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class ROSWebBridge(Node):
    def __init__(self):
        super().__init__('ros_web_bridge')

        # Default status values
        self.amr_status = "Waiting..."
        self.manip_status = "Waiting..."
        self.system_status = "Waiting..."

        # ROS subscriptions
        self.create_subscription(String, '/system/ready', self.ready_callback, 10)
        self.create_subscription(String, '/system/state', self.system_callback, 10)

        # ROS publisher
        self.pallet_pub = self.create_publisher(Int32, '/pallet_request', 10)

        self.amr_last_seen = 0
        self.manip_last_seen = 0
        self.task_state = "idle"


    def ready_callback(self, msg):
        now = time.time()
        if msg.data == "amr_ready":
            self.amr_status = "🟢 Ready"
            self.amr_last_seen = now
        elif msg.data == "manip_ready":
            self.manip_status = "🟢 Ready"
            self.manip_last_seen = now
                
    def system_callback(self, msg):
        self.system_status = msg.data
        if "Task complete" in msg.data:
            self.task_state = "idle"

    def send_pallet_request(self, pallet_number: int):
        msg = Int32()
        msg.data = pallet_number
        self.pallet_pub.publish(msg)


# Initialize ROS node and spin in a thread
rclpy.init()
bridge = ROSWebBridge()
threading.Thread(target=rclpy.spin, args=(bridge,), daemon=True).start()

@app.get("/status")
def get_status():
    now = time.time()
    if now - bridge.amr_last_seen > 5:
        bridge.amr_status = "🔴 Disconnected"
    if now - bridge.manip_last_seen > 5:
        bridge.manip_status = "🔴 Disconnected"

    return {
        "amr": bridge.amr_status,
        "manipulator": bridge.manip_status,
        "system": bridge.system_status
    }


@app.post("/pallet/{number}")
def set_pallet(number: int):
    if bridge.task_state != "idle":
        return {"success": False, "message": "🚧 System is busy. Wait until current task finishes."}

    if 1 <= number <= 10:
        bridge.send_pallet_request(number)
        bridge.task_state = "busy"
        return {"success": True, "message": f"Pallet {number} request sent."}
    return {"success": False, "message": "Invalid pallet number. Please use 1–10."}
