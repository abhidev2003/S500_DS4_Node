import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Header, String
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleAttitude
import math

class HeartNode(Node):
    def __init__(self):
        super().__init__('skypal_core_heart_node')
        
        # Configure QoS profile for publishing and subscribing to px4_msgs
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriptions from VPN (Controller)
        self.cmd_sub = self.create_subscription(Twist, '/skypal/cmd_vel', self.cmd_callback, 10)
        self.heartbeat_sub = self.create_subscription(Header, '/skypal/heartbeat', self.heartbeat_callback, 10)
        self.sys_cmd_sub = self.create_subscription(String, '/skypal/sys_command', self.sys_cmd_callback, 10)
        
        # Subscriptions from PX4 (Agent)
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.attitude_sub = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_callback, qos_profile)
        
        # Publishers to PX4
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        # State
        self.last_heartbeat_time = self.get_clock().now()
        self.last_cmd = Twist()
        self.failsafe_triggered = False
        self.offboard_engaged = False
        self.max_latency_sec = 0.5  # 500ms
        
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_STANDBY
        self.current_yaw = 0.0
        
        # Timers
        self.monitor_timer = self.create_timer(0.1, self.monitor_connection)     # 10Hz Monitor
        self.offboard_timer = self.create_timer(0.05, self.publish_offboard_heartbeat) # 20Hz Setpoint Publisher
        
        self.get_logger().info('Skypal Core Heart Node Initialized. Ready for Offboard Commands.')

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def attitude_callback(self, msg):
        q = msg.q
        self.current_yaw = math.atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2]*q[2] + q[3]*q[3]))

    def heartbeat_callback(self, msg):
        now = self.get_clock().now()
        stamp_time = rclpy.time.Time.from_msg(msg.stamp)
        
        latency = (now - stamp_time).nanoseconds / 1e9
        
        if latency > self.max_latency_sec:
            if not self.failsafe_triggered:
                self.get_logger().warning(f'High Latency Detected: {latency:.3f}s. Triggering Failsafe Hover!')
                self.failsafe_triggered = True
        else:
            if self.failsafe_triggered:
                self.get_logger().info(f'Connection restored. Latency: {latency:.3f}s. Resuming normal operations.')
                self.failsafe_triggered = False
                
        self.last_heartbeat_time = now

    def cmd_callback(self, msg):
        self.last_cmd = msg

    def sys_cmd_callback(self, msg):
        command = msg.data
        if command == "arm_offboard":
            self.get_logger().info("Received Arm+Offboard command from Controller.")
            self.arm()
            self.engage_offboard_mode()
            self.offboard_engaged = True
        elif command == "land":
            self.get_logger().info("Received Land command from Controller.")
            self.land()
            self.offboard_engaged = False

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")
        
    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def monitor_connection(self):
        now = self.get_clock().now()
        time_since_last_heartbeat = (now - self.last_heartbeat_time).nanoseconds / 1e9
        
        if time_since_last_heartbeat > self.max_latency_sec and not self.failsafe_triggered:
             self.get_logger().error(f'Connection Lost! No heartbeat for {time_since_last_heartbeat:.3f}s. Triggering PX4 Failsafe Hover!')
             self.failsafe_triggered = True

    def publish_offboard_heartbeat(self):
        # Must publish OffboardControlMode continuously at 20Hz to maintain PX4 connection
        # PX4 REQUIRES this stream to exist BEFORE it will accept the switch to Offboard mode!
        mode_msg = OffboardControlMode()
        mode_msg.position = False
        mode_msg.velocity = True
        mode_msg.acceleration = False
        mode_msg.attitude = False
        mode_msg.body_rate = False
        mode_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(mode_msg)
        
        # Publish Trajectory Setpoint
        setpoint_msg = TrajectorySetpoint()
        setpoint_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        
        if self.failsafe_triggered:
            # Hover in place exactly where the drone is (Zero velocity on all axes)
            setpoint_msg.velocity = [0.0, 0.0, 0.0]
            setpoint_msg.yawspeed = 0.0
        else:
            # Body Frame inputs from controller
            v_forward = self.last_cmd.linear.x
            v_right = -self.last_cmd.linear.y
            v_down = -self.last_cmd.linear.z
            
            # Rotate body velocities to Earth (NED) frame based on current yaw
            v_north = v_forward * math.cos(self.current_yaw) - v_right * math.sin(self.current_yaw)
            v_east = v_forward * math.sin(self.current_yaw) + v_right * math.cos(self.current_yaw)
            
            setpoint_msg.velocity = [v_north, v_east, v_down]
            setpoint_msg.yawspeed = -self.last_cmd.angular.z  # FLU CCW to NED CW yaw
            
        # Set Position array to NaN so the FMU ignores it and acts solely on velocity
        setpoint_msg.position = [float('nan'), float('nan'), float('nan')]
        setpoint_msg.acceleration = [float('nan'), float('nan'), float('nan')]
        setpoint_msg.jerk = [float('nan'), float('nan'), float('nan')]
        setpoint_msg.yaw = float('nan')
        
        self.trajectory_setpoint_publisher.publish(setpoint_msg)

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
