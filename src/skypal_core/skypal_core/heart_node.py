import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Header, String
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLocalPosition, VehicleAttitude
import math
import os
import time

class HeartNode(Node):
    def __init__(self):
        super().__init__('skypal_core_heart_node')
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriptions from Skypal Controller
        self.cmd_sub = self.create_subscription(Twist, '/skypal/cmd_vel', self.cmd_callback, qos_profile)
        self.sys_cmd_sub = self.create_subscription(String, '/skypal/sys_command', self.sys_cmd_callback, 10)
        self.heartbeat_sub = self.create_subscription(Header, '/skypal/heartbeat', self.heartbeat_callback, 10)
        
        # Providers for Position Integration
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0
        
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.target_yaw = 0.0
        
        # Velocity Smoothing (Low-Pass Filter)
        self.smooth_v_body_x = 0.0
        self.smooth_v_body_y = 0.0
        self.smooth_v_down = 0.0
        self.smooth_v_yaw = 0.0
        self.alpha_v = 0.4 # EMA Smoothing Factor (0.0=Static to 1.0=Instant)
        
        self.local_pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_pos_callback, qos_profile)
        self.attitude_sub = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_callback, qos_profile)
        # Publishers to PX4 (XRCE-DDS)
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        
        # State Variables
        self.last_heartbeat_time = self.get_clock().now()
        self.last_cmd = Twist()
        self.failsafe_triggered = False
        self.offboard_engaged = False
        self.max_latency_sec = 2.0 # Increased for Mobile Network (JIOPAL) Jitter Tolerance
        self.dt = 0.05 # 20Hz loop
        
        self.nav_state = 14 # VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = 1 # VehicleStatus.ARMING_STATE_DISARMED
        
        # Timers
        self.monitor_timer = self.create_timer(0.1, self.monitor_connection)     # 10Hz Monitor
        self.offboard_timer = self.create_timer(self.dt, self.publish_offboard_heartbeat) # 20Hz Setpoint Publisher
        
        self.declare_parameter('is_sim', True)
        self.is_sim = self.get_parameter('is_sim').value
        
        log_type = "sim" if self.is_sim else "real"
        log_dir = os.path.expanduser(f"~/skypal_ws/logs/{log_type}")
        os.makedirs(log_dir, exist_ok=True)
        filename = time.strftime("%Y-%m-%d_%H-%M-%S_Heart_PX4_Pos.log")
        self.log_file = open(os.path.join(log_dir, filename), 'a')
        
        self.log("INFO", f"Skypal Core PX4 Heart Node Initialized ({log_type.upper()} Mode - POSITION). Ready for Commands.")

    def log(self, level, msg):
        log_str = f"[{time.strftime('%H:%M:%S.%f')[:-3]}] [{level}] {msg}\n"
        self.log_file.write(log_str)
        self.log_file.flush()
        if level == "INFO": self.get_logger().info(msg)
        elif level == "WARN": self.get_logger().warning(msg)
        elif level == "ERROR": self.get_logger().error(msg)
        
    def local_pos_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        # Removed self.current_yaw = msg.heading (unreliable in SITL)

    def attitude_callback(self, msg):
        # Extract Euler yaw from PX4 quaternion (q = [w, x, y, z])
        q0 = msg.q[0]
        q1 = msg.q[1]
        q2 = msg.q[2]
        q3 = msg.q[3]
        self.current_yaw = math.atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3))

    def status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def heartbeat_callback(self, msg):
        now = self.get_clock().now()
        # We exclusively update the latest receipt time.
        # We CANNOT calculate absolute one-way latency via (now - msg.stamp)
        # over the VPN because the Pi and Laptop clocks are not perfectly NTP synchronized.
        # This caused false>2.0s latency triggers instantly locking the drone into a failsafe hover.
        # The `monitor_connection` method correctly handles interval dropouts locally.
        self.last_heartbeat_time = now

    def cmd_callback(self, msg):
        self.last_cmd = msg

    def sys_cmd_callback(self, msg):
        command = msg.data
        if command == "arm_offboard":
            self.log("INFO", "Received Arm command. Requesting Offboard Mode and Arming PX4.")
            # Latch position right exactly when we arm to prevent flyaways
            self.target_x = self.current_x
            self.target_y = self.current_y
            self.target_z = self.current_z
            self.target_yaw = self.current_yaw
            self.engage_offboard()
            self.arm()
            self.offboard_engaged = True
        elif command == "land":
            self.log("INFO", "Received Land command. Switching to Auto-Land Native Mode.")
            self.land()
            self.offboard_engaged = False

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def arm(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.log("INFO", "Arm command sent to PX4.")

    def engage_offboard(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.log("INFO", "Offboard mode command sent.")

    def land(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.log("INFO", "Land command sent.")

    def monitor_connection(self):
        now = self.get_clock().now()
        time_since_last_heartbeat = (now - self.last_heartbeat_time).nanoseconds / 1e9
        
        if time_since_last_heartbeat > self.max_latency_sec:
            if not self.failsafe_triggered:
                 self.log("ERROR", f"Connection Lost! No heartbeat for {time_since_last_heartbeat:.3f}s. Triggering Failsafe Hover!")
                 self.failsafe_triggered = True
        else:
            if self.failsafe_triggered:
                 self.log("INFO", f"Connection Restored ({time_since_last_heartbeat:.3f}s). Failsafe Disabled.")
                 self.failsafe_triggered = False

    def publish_offboard_heartbeat(self):
        # 1. Provide OffboardControlMode Signal
        mode_msg = OffboardControlMode()
        mode_msg.position = False
        mode_msg.velocity = True
        mode_msg.acceleration = False
        mode_msg.attitude = False
        mode_msg.body_rate = False
        mode_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(mode_msg)
        
        # 2. Provide TrajectorySetpoint Signal
        msg = TrajectorySetpoint()
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float('nan')
        
        if self.failsafe_triggered or not self.offboard_engaged:
            # Hover in place exactly where the drone is
            self.smooth_v_body_x = 0.0
            self.smooth_v_body_y = 0.0
            self.smooth_v_down = 0.0
            self.smooth_v_yaw = 0.0
            msg.velocity = [0.0, 0.0, 0.0]
            msg.yawspeed = 0.0
        else:
            # Body Frame Control Logic:
            # ROS 2 Twist is FLU (Forward, Left, Up)
            # PX4 Body is FRD (Forward, Right, Down)
            
            if self.is_sim:
                # Standard Mapping: +linear.x is Forward, +linear.y is Left
                # PX4 Body FRD: +X is Forward, +Y is Right
                target_v_body_x = self.last_cmd.linear.x    
                target_v_body_y = -self.last_cmd.linear.y   
            else:
                # AXIS SWAP: Hardware testing revealed a 90-degree physical offset.
                # Stick Left (+linear.y) goes Left (+v_body_x produced Left)
                # Stick Forward (+linear.x) goes Forward (+v_body_y produced Forward)
                target_v_body_x = self.last_cmd.linear.y    
                target_v_body_y = self.last_cmd.linear.x   
            
            target_v_down = -self.last_cmd.linear.z     # Stick Up      -> Body -Z (Up is negative Down)
            target_v_yaw = -self.last_cmd.angular.z     # CCW spin mapped to PX4 yawspeed
            
            # Apply Exponential Moving Average (EMA) to smooth out network jitter
            self.smooth_v_body_x += self.alpha_v * (target_v_body_x - self.smooth_v_body_x)
            self.smooth_v_body_y += self.alpha_v * (target_v_body_y - self.smooth_v_body_y)
            self.smooth_v_down += self.alpha_v * (target_v_down - self.smooth_v_down)
            self.smooth_v_yaw += self.alpha_v * (target_v_yaw - self.smooth_v_yaw)
            
            # Apply 2D Trigonometric Rotation Matrix to convert Body FRD to Geographic Local NED
            yaw = self.current_yaw
            v_north = self.smooth_v_body_x * math.cos(yaw) - self.smooth_v_body_y * math.sin(yaw)
            v_east = self.smooth_v_body_x * math.sin(yaw) + self.smooth_v_body_y * math.cos(yaw)
            
            msg.velocity = [v_north, v_east, self.smooth_v_down]
            msg.yawspeed = self.smooth_v_yaw


        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HeartNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
