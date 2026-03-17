import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Header, String
import os
import time

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class ControllerNode(Node):
    def __init__(self):
        super().__init__('skypal_controller_node')
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publisher for Heartbeat
        self.heartbeat_pub = self.create_publisher(Header, '/skypal/heartbeat', 10)
        
        # Publisher for Velocity Commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/skypal/cmd_vel', qos_profile)
        
        # Publisher for System Commands (Arm, Offboard, Land)
        self.sys_cmd_pub = self.create_publisher(String, '/skypal/sys_command', 10)
        
        # Subscriber to standard ROS joy node
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Timer to publish heartbeat at 50Hz (0.02s)
        self.heartbeat_timer = self.create_timer(0.02, self.publish_heartbeat)
        
        # Determine Sim vs Real mode from ROS Args
        self.declare_parameter('is_sim', True)
        self.is_sim = self.get_parameter('is_sim').value
        
        # Setup Custom Diagnostic Logging
        log_type = "sim" if self.is_sim else "real"
        log_dir = os.path.expanduser(f"~/skypal_ws/logs/{log_type}")
        os.makedirs(log_dir, exist_ok=True)
        filename = time.strftime("%Y-%m-%d_%H-%M-%S_Controller.log")
        self.log_file = open(os.path.join(log_dir, filename), 'a')
        
        # Button states to detect press
        self.last_buttons = []
        
        self.log("INFO", f"Skypal Controller Node Initialized ({log_type.upper()} Mode). Publishing heartbeat at 50Hz.")

    def log(self, level, msg):
        log_str = f"[{time.strftime('%H:%M:%S.%f')[:-3]}] [{level}] {msg}\n"
        self.log_file.write(log_str)
        self.log_file.flush()
        if level == "INFO": self.get_logger().info(msg)
        elif level == "WARN": self.get_logger().warning(msg)
        elif level == "ERROR": self.get_logger().error(msg)

    def joy_callback(self, msg):
        # DualShock button 9 is usually 'Options'. We use it to Arm and switch to Offboard
        # DualShock button 8 is usually 'Share'. We use it to Land
        if len(self.last_buttons) > 0:
            # Debug: Print any button that is pressed to help identify the index
            for i, (new_val, old_val) in enumerate(zip(msg.buttons, self.last_buttons)):
                if new_val == 1 and old_val == 0:
                    self.get_logger().info(f"Button {i} pressed!")

            # Detect rising edge on Options button (Usually 9)
            if len(msg.buttons) > 9 and msg.buttons[9] == 1 and self.last_buttons[9] == 0:
                self.log("INFO", "Options pressed: Sending 'arm_offboard' command")
                sys_msg = String()
                sys_msg.data = "arm_offboard"
                self.sys_cmd_pub.publish(sys_msg)
            
            # Detect rising edge on Share button
            if len(msg.buttons) > 8 and msg.buttons[8] == 1 and self.last_buttons[8] == 0:
                self.log("INFO", "Share pressed: Sending 'land' command")
                sys_msg = String()
                sys_msg.data = "land"
                self.sys_cmd_pub.publish(sys_msg)
                
        self.last_buttons = msg.buttons
        
        twist = Twist()
        # Scale inputs (max 5.0 m/s or rad/s for responsiveness in sim)
        max_speed = 5.0
        
        if len(msg.axes) >= 5:
            # Mode 2 RC Default Mapping for DualShock 4
            
            # Deadzone filter to prevent violent initialization spikes from triggering PX4 Runaway Failsafe
            def apply_deadzone(value, deadzone=0.15):
                return 0.0 if abs(value) < deadzone else value

            # Right Stick U/D -> Pitch Forward (Negative is usually UP on joystick, so we invert if needed)
            twist.linear.x = apply_deadzone(msg.axes[4]) * max_speed
            
            # Right Stick L/R -> Roll Left
            twist.linear.y = apply_deadzone(msg.axes[3]) * max_speed
            
            # Left Stick U/D -> Throttle Up (Altitude)
            twist.linear.z = apply_deadzone(msg.axes[1]) * max_speed
            
            # Left Stick L/R -> Yaw
            twist.angular.z = apply_deadzone(msg.axes[0]) * max_speed
        
        self.cmd_vel_pub.publish(twist)

    def publish_heartbeat(self):
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        msg.frame_id = "skypal_controller"
        self.heartbeat_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
