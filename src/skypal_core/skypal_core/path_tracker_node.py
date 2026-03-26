#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition, TrajectorySetpoint
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import math
import time

class PathTrackerNode(Node):
    def __init__(self):
        super().__init__('path_tracker_node')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.local_pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.pos_callback, qos_profile)
        self.sys_cmd_sub = self.create_subscription(String, '/skypal/sys_command', self.sys_cmd_callback, 10)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, '/skypal/autonomous_trajectory', 10)
        
        self.path = [] # List of (N, E, D) -> (X, Y, Z) tuples
        self.record_threshold = 1.0 # Meters minimum between logged breadcrumbs
        self.is_rtl = False
        self.target_index = -1
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        
        # 20Hz trajectory playback
        self.dt = 0.05 
        self.timer = self.create_timer(self.dt, self.execute_rtl)
        
        self.get_logger().info("Skypal Path Tracker Node started. Logging flight path...")

    def pos_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        
        if not self.is_rtl:
            if len(self.path) == 0:
                self.path.append((msg.x, msg.y, msg.z))
            else:
                last_p = self.path[-1]
                dist = math.sqrt((msg.x - last_p[0])**2 + (msg.y - last_p[1])**2 + (msg.z - last_p[2])**2)
                if dist > self.record_threshold:
                    self.path.append((msg.x, msg.y, msg.z))
        
    def sys_cmd_callback(self, msg):
        if msg.data == "nav_rtl":
            if not self.is_rtl:
                self.get_logger().info(f"RTL Triggered! Engaged Trace-Back Protocol. Route has {len(self.path)} steps.")
                self.is_rtl = True
                if len(self.path) > 0:
                    self.target_index = len(self.path) - 1

    def execute_rtl(self):
        # We only govern flight velocity during the active backtracking RTL sequence
        if not self.is_rtl:
            return
            
        if self.target_index < 0:
            # We reached index 0 smoothly. Hover indefinitely exactly at the home origin.
            setpoint = TrajectorySetpoint()
            setpoint.position = [float('nan'), float('nan'), float('nan')]
            setpoint.velocity = [0.0, 0.0, 0.0]
            setpoint.yawspeed = 0.0
            setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.traj_pub.publish(setpoint)
            return
            
        target = self.path[self.target_index]
        dx = target[0] - self.current_x
        dy = target[1] - self.current_y
        dz = target[2] - self.current_z
        
        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        
        # When within a 1 meter radius bubble, advance backwards to the previous mapped vertex
        if dist < 1.0: 
            self.target_index -= 1
            
            if self.target_index < 0:
                self.get_logger().info("Successfully arrived at Home Origin. Holding hover!")
                return
                
            target = self.path[self.target_index]
            dx = target[0] - self.current_x
            dy = target[1] - self.current_y
            dz = target[2] - self.current_z
            dist = math.sqrt(dx**2 + dy**2 + dz**2)

        fly_speed = 3.0 # Meters per second back-flight
        if dist > 0:
            vx = (dx / dist) * fly_speed
            vy = (dy / dist) * fly_speed
            vz = (dz / dist) * fly_speed
        else:
            vx, vy, vz = 0.0, 0.0, 0.0
            
        yaw = math.atan2(vy, vx)
        
        # Dispatch the localized speed vectors to the Heart node's multiplexing listener
        setpoint = TrajectorySetpoint()
        setpoint.position = [float('nan'), float('nan'), float('nan')]
        setpoint.velocity = [vx, vy, vz]
        setpoint.yaw = yaw
        setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        self.traj_pub.publish(setpoint)

def main(args=None):
    rclpy.init(args=args)
    node = PathTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
