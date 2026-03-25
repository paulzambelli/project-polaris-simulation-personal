import math
import rclpy
import logging, os
from rclpy.node import Node
from pymavlink import mavutil
from datetime import datetime
from std_msgs.msg import Int16MultiArray, String
from sensor_msgs.msg import (
    Imu,  # ATTITUDE
    BatteryState,  # BATTERY_STATUS
    FluidPressure,  # SCALED_PRESSURE (depth)
)

# specifies the directory where logs are saved and the name of the log files
log_dir = os.path.expanduser("~/polaris_logs")
os.makedirs(log_dir, exist_ok=True)
log_file = os.path.join(log_dir, f"mavlink_{datetime.now():%Y%m%d_%H%M%S}.log")


class DualLogger:
    def __init__(self, ros_logger, file_logger):
        self._ros = ros_logger
        self._file = file_logger

    def info(self, msg):
        self._ros.info(msg)
        self._file.info(msg)

    def debug(self, msg):
        self._ros.debug(msg)
        self._file.debug(msg)

    def warning(self, msg):
        self._ros.warning(msg)
        self._file.warning(msg)

    def error(self, msg):
        self._ros.error(msg)
        self._file.error(msg)


class MavlinkBridgeSender(Node):
    """
    This node is supposed to publish mavlink data directly from the pixhawk to ROS2 topics and so that it can be heart by the output monitor
    """

    def __init__(self):
        super().__init__("mavlink_bridge_publisher")

        self._file_logger = logging.getLogger(
            "mavlink"
        )  # creates or gets logger instance

        # set level defines from what message type onwards the message is logged. the different levels are:
        # Logging levels (lowest → highest):
        # DEBUG    = detailed diagnostic data (high-frequency sensor + internal state)
        # INFO     = normal operational messages (mode changes, summaries)
        # WARNING  = unexpected situations that do not stop operation
        # ERROR    = recoverable failures
        # CRITICAL = unrecoverable failures; system may be unusable
        self._file_logger.setLevel(logging.INFO)

        # the handler actually writes to the specified file
        file_handler = logging.FileHandler(log_file)
        formatter = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
        file_handler.setFormatter(formatter)
        self._file_logger.addHandler(file_handler)

        self.ros_logger = self.get_logger()  # get_logger is the ros logger object

        self.logger = DualLogger(self.ros_logger, self._file_logger)

        # SITL: SERIAL0 default is tcp:127.0.0.1:5760 (see ArduPilot UART / console doc).
        # Keep ros2_receiver on a different port (default 5762 / SERIAL1) to avoid two clients on one serial.
        mavlink_url = os.getenv("MAVLINK_PUBLISHER_URL", "tcp:127.0.0.1:5760")
        mavlink_baud = int(os.getenv("MAVLINK_PUBLISHER_BAUD", "57600"))
        self.port = mavutil.mavlink_connection(mavlink_url, baud=mavlink_baud)

        self.port.wait_heartbeat()
        self.logger.info(f"Heartbeat received from system {self.port.target_system}")

        self.heartbeat_publisher = self.create_publisher(
            String, "/pixhawk/heartbeat", 10)

        self.attitude_publisher = self.create_publisher(
            Imu, "/pixhawk/attitude", 10)

        self.rc_channel_publisher = self.create_publisher(
            Int16MultiArray, "/pixhawk/rc_channels", 10)

        self.battery_publisher = self.create_publisher(
            BatteryState, "/pixhawk/battery", 10
        )

        self.scaled_pressure_publisher = self.create_publisher(
            FluidPressure, "/pixhawk/scaled_pressure", 10
        )

        self.manual_control_publisher = self.create_publisher(
            Int16MultiArray, "/pixhawk/out/manual_control", 10
        )

        self.timer = self.create_timer(0.5, self.mavlink_callback)

        # Request MANUAL_CONTROL messages at 10 Hz
        self.logger.info("Requesting MANUAL_CONTROL message stream from Pixhawk...")
        self.port.mav.command_long_send(
            self.port.target_system,
            self.port.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,  # confirmation
            mavutil.mavlink.MAVLINK_MSG_ID_MANUAL_CONTROL,  # message ID = 69
            100000,  # interval in microseconds (100ms = 10Hz)
            0, 0, 0, 0, 0
        )
        self.logger.info(
            f"MANUAL_CONTROL request sent (msg_id={mavutil.mavlink.MAVLINK_MSG_ID_MANUAL_CONTROL}, interval=100ms)"
        )

    def mavlink_callback(self):
        """Timer callback - drains all buffered MAVLink messages and routes them"""
        # Process ALL available messages in the buffer (not just one)
        while True:
            msg = self.port.recv_match(blocking=False)
            if msg is None:
                break  # No more messages in buffer

            if msg is not None:
                self.logger.debug(f"Received: {msg.get_type()}")

                if msg.get_type() == "HEARTBEAT":
                    self.handle_heartbeat(msg)
                elif msg.get_type() == "ATTITUDE":
                    self.handle_attitude(msg)
                elif msg.get_type() == "RC_CHANNELS":
                    self.handle_rc_channels(msg)
                elif msg.get_type() == "BATTERY_STATUS":
                    self.handle_battery(msg)
                elif msg.get_type() == "SCALED_PRESSURE2":
                    self.handle_scaled_pressure(msg)
                elif msg.get_type() == "MANUAL_CONTROL":
                    self.handle_manual_control(msg)

        

    def handle_heartbeat(self, msg):
        """Process HEARTBEAT message and publish to ROS2"""
        # Filter: only process heartbeats from actual autopilots, not GCS or other components
        # MAV_AUTOPILOT_INVALID (8) means it's not an autopilot (e.g., GCS, companion computer)
        if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            return  # Skip non-autopilot heartbeats

        mode_mapping = mavutil.mode_mapping_sub
        mode = mode_mapping.get(
            msg.custom_mode, f"UNKNOWN({msg.custom_mode})"
        )
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

        ros_msg = String()
        ros_msg.data = f"mode={mode};armed={int(armed)};system_status={msg.system_status}"
        self.heartbeat_publisher.publish(ros_msg)
        self.logger.debug(
            f"Published Heartbeat: Status={msg.system_status}, Mode={mode}, Armed={armed}"
        )

    def handle_attitude(self, msg):
        """Process ATTITUDE message and publish to ROS2"""
        ros_msg = Imu()

        # Convert Euler angles (radians) to Quaternion
        roll = msg.roll
        pitch = msg.pitch
        yaw = msg.yaw

        # Euler to Quaternion conversion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        ros_msg.orientation.w = cr * cp * cy + sr * sp * sy
        ros_msg.orientation.x = sr * cp * cy - cr * sp * sy
        ros_msg.orientation.y = cr * sp * cy + sr * cp * sy
        ros_msg.orientation.z = cr * cp * sy - sr * sp * cy

        self.attitude_publisher.publish(ros_msg)
        self.logger.debug(
            f"Published Attitude: Roll={roll:.2f}, Pitch={pitch:.2f}, Yaw={yaw:.2f}"
        )

    def handle_rc_channels(self, msg):
        """Process RC_CHANNELS message and publish to ROS2"""
        ros_msg = Int16MultiArray()
        ros_msg.data = [msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw]

        self.rc_channel_publisher.publish(ros_msg)
        self.logger.debug(f"Published RC: {ros_msg.data}")

    def handle_battery(self, msg):
        """Process BATTERY_STATUS message and publish to ROS2"""
        ros_msg = BatteryState()
        # current_battery is in 10*mA (centiamperes), divide by 100 to get Amperes
        ros_msg.current = float(msg.current_battery) / 100.0
        # battery_remaining is percentage (0-100), ROS2 expects 0.0-1.0
        ros_msg.percentage = float(msg.battery_remaining) / 100.0

        self.battery_publisher.publish(ros_msg)
        self.logger.debug(
            f"Published Battery: Current={ros_msg.current:.2f}A, Remaining={ros_msg.percentage:.0%}"
        )

    def handle_scaled_pressure(self, msg):
        """SCALED_PRESSURE2: MAVLink press_abs is absolute pressure in hPa; ROS FluidPressure is Pa (×100)."""
        ros_msg = FluidPressure()
        pa = float(msg.press_abs) * 100.0
        ros_msg.fluid_pressure = pa
        self.scaled_pressure_publisher.publish(ros_msg)
        # Absurd values usually mean SITL/external baro misconfiguration (see sub.parm BARO_EXT_BUS).
        self.logger.debug(f"Published pressure (abs): {pa:.1f} Pa (press_abs hPa={msg.press_abs})")

    def handle_manual_control(self, msg):
        """Process MANUAL_CONTROL message and publish to ROS2"""
        self.logger.debug("manual control callback triggered")
        ros_msg = Int16MultiArray()
        ros_msg.data = [msg.x, msg.y, msg.z, msg.r, msg.buttons, msg.s, msg.t]
        self.manual_control_publisher.publish(ros_msg)
        self.logger.debug(
            f"Published Manual Control: x={msg.x}, y={msg.y}, z={msg.z}, r={msg.r}, s={msg.s}, t={msg.t}"
        ) 


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkBridgeSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, shutting down mavlink_bridge_publisher")
    finally:
        try:
            if hasattr(node, "port") and node.port is not None:
                node.port.close()
        except Exception as exc:
            node.get_logger().warning(f"Failed to close MAVLink port cleanly: {exc}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
