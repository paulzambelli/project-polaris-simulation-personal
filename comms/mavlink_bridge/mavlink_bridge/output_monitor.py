import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray
from sensor_msgs.msg import (
    Imu,  # ATTITUDE
    BatteryState,  # BATTERY_STATUS
    FluidPressure,  # SCALED_PRESSURE (depth)
)
from geometry_msgs.msg import Twist # Needs to be adapted for sending cmd_vel


# Standalone defaults for simulation / docker runtime.
LOG_DIR = "~/polaris_logs"
SUB_QOS_DEPTH = 10


class OutputMonitor(Node):
    # MAV_STATE mapping (system_status values)
    SYSTEM_STATUS_MAP = {
        0: "UNINIT",
        1: "BOOT",
        2: "CALIBRATING",
        3: "STANDBY",
        4: "ACTIVE",
        5: "CRITICAL",
        6: "EMERGENCY",
        7: "POWEROFF",
        8: "FLIGHT_TERMINATION",
    }

    def __init__(self):
        super().__init__("output_monitor")

        # Initialize state variables
        self.mode = "UNKNOWN"
        self.armed = False
        self.system_status = "UNKNOWN"
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.rc_channels = []
        self.cmd_velocity = None  # set by /pixhawk/cmd_vel; None = no samples yet
        self.pressure_diff = 0.0

        # Create subscriptions
        self.create_subscription(String, "/pixhawk/heartbeat", self.heartbeat_cb, 10)
        self.create_subscription(Imu, "/pixhawk/attitude", self.attitude_cb, 10)
        self.create_subscription(Int16MultiArray, "/pixhawk/rc_channels", self.rc_cb, 10)
        self.create_subscription(Twist, "/pixhawk/cmd_vel", self.cmd_vel_cb, SUB_QOS_DEPTH)
        self.create_subscription(
            FluidPressure, "/pixhawk/scaled_pressure", self.pressure_cb, 10
        )

        self.timer = self.create_timer(0.5, self.print_dashboard)  # 5Hz refresh rate

    def heartbeat_cb(self, msg):
        fields = {}
        for item in msg.data.split(";"):
            if "=" in item:
                key, value = item.split("=", 1)
                fields[key.strip()] = value.strip()
        self.mode = fields.get("mode", "UNKNOWN")
        self.armed = fields.get("armed", "0") == "1"
        status_raw = int(fields.get("system_status", "-1"))
        self.system_status = self.SYSTEM_STATUS_MAP.get(status_raw, f"UNKNOWN({status_raw})")

    def attitude_cb(self, msg):
        # Convert quaternion back to Euler angles for display
        w = msg.orientation.w
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z

        # Quaternion to Euler angles conversion
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        self.roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            self.pitch = math.copysign(math.pi / 2, sinp)
        else:
            self.pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def rc_cb(self, msg):
        self.rc_channels = msg.data

    def pressure_cb(self, msg):
        self.pressure_diff = msg.fluid_pressure

    def cmd_vel_cb(self, msg):
        self.cmd_velocity = (
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z,
        )

    # this function actually print the dashboard
    def print_dashboard(self):
        # Clear screen code
        print("\033[H\033[J", end="")

        print("=== MAVLINK BRIDGE MONITOR ===")

        print(f"Mode: {self.mode}")
        print(f"Armed: {self.armed}")
        print(f"System Status: {self.system_status}")

        print(f"Roll: {self.roll}")
        print(f"Pitch: {self.pitch}")
        print(f"Yaw: {self.yaw}")

        # From SCALED_PRESSURE(1..3) via mavlink_publisher (valid press_abs only; hPa×100 -> Pa).
        print(f"Pressure (abs est., Pa): {self.pressure_diff}")

        print(f"RC Channels: {list(self.rc_channels)}")
        cv = self.cmd_velocity
        print(
            "cmd_vel (lin x,y,z | ang x,y,z): "
            f"{list(cv) if cv is not None else 'no msgs yet'}"
        )
        print("============================")


def main(args=None):
    rclpy.init(args=args)
    node = OutputMonitor()
    try:
        rclpy.spin(node)  # keeps the node running
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, shutting down output_monitor")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
