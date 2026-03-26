#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, VehicleGlobalPosition, VehicleLocalPosition, DistanceSensor
from std_msgs.msg import String

import math
import numpy as np

class SkyPalMissionCommander(Node):
    def __init__(self):
        super().__init__('skypal_mission_commander')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/skypal/autonomous_trajectory', 10)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        self.global_pos_sub = self.create_subscription(VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.global_pos_cb, 10)
        self.local_pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_pos_cb, 10)
        self.distance_sub = self.create_subscription(DistanceSensor, '/fmu/out/distance_sensor', self.distance_cb, qos_profile)
        
        # New Local UI Mission Dispatcher
        self.local_mission_sub = self.create_subscription(String, '/skypal/local_mission', self.local_mission_cb, 10)

        self.get_logger().info("🛡️ Drone Autonomy Core offline and eagerly awaiting UI map dispatch.")

        # Mission State
        self.state = 'IDLE' # IDLE -> PIVOT -> GLIDE -> LANDING -> AWAITING_TAKEOFF -> (IDLE)
        self.waypoints = []
        self.current_wp_index = 0
        self.landing_timestamp = 0.0
        self.is_offboard = False

        self.home_lat = None
        self.home_lon = None
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_heading = 0.0
        
        # Lunar Lidar failsafe states
        self.lidar_distance = float('inf')
        self.lidar_active = False

        self.target_ned_x = 0.0
        self.target_ned_y = 0.0
        
        # High-frequency timer for PX4 Offboard (10Hz)
        self.timer = self.create_timer(0.1, self.offboard_heartbeat)

    def local_mission_cb(self, msg):
        if self.state != 'IDLE':
            self.get_logger().warn("Mission physically in progress! Ignoring local dispatch until current queue clears.")
            return
            
        parts = [float(x) for x in msg.data.split(',')]
        if len(parts) == 4 and self.home_lat is not None:
            self.get_logger().info("📥 Local Multi-Waypoint Mission Intercepted via GUI Map!")
            send_lat, send_lon, recv_lat, recv_lon = parts
            
            # Mission Phase Plan
            self.waypoints = [
                (send_lat, send_lon),       # Wp 0: Send Location
                (recv_lat, recv_lon),       # Wp 1: Receive Location
                (self.home_lat, self.home_lon) # Wp 2: Returning to the initial Origin base
            ]
            self.current_wp_index = 0
            self.advance_waypoint()

    def advance_waypoint(self):
        if self.current_wp_index >= len(self.waypoints):
            self.state = 'IDLE'
            self.get_logger().info("🎉 Complete Mission logic loop finished!")
            return
            
        target_lat, target_lon = self.waypoints[self.current_wp_index]
        self.target_ned_x, self.target_ned_y = self.wgs84_to_ned(target_lat, target_lon)
        self.get_logger().info(f"Target Acquired (Waypoint {self.current_wp_index}): NED X={self.target_ned_x:.2f}m, Y={self.target_ned_y:.2f}m")
        
        # Spool Up
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0) # Offboard
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.is_offboard = True
        self.state = 'PIVOT'

    def distance_cb(self, msg):
        if msg.orientation == 1 or True: 
            self.lidar_distance = msg.current_distance
            self.lidar_active = True

    def global_pos_cb(self, msg):
        self.current_lat = msg.lat
        self.current_lon = msg.lon
        
        # Retain original spawn origin as Base Station fallback
        if self.home_lat is None and msg.lat != 0.0:
            self.home_lat = msg.lat
            self.home_lon = msg.lon
            self.get_logger().info(f"📍 Home Base Station physically Locked: {self.home_lat}, {self.home_lon}")

    def local_pos_cb(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        self.current_heading = msg.heading

    def wgs84_to_ned(self, target_lat, target_lon):
        r_earth = 6378137.0
        lat_rad = math.radians(self.home_lat)
        lon_rad = math.radians(self.home_lon)
        target_lat_rad = math.radians(target_lat)
        target_lon_rad = math.radians(target_lon)
        
        d_lat = target_lat_rad - lat_rad
        d_lon = target_lon_rad - lon_rad
        
        x = r_earth * d_lat
        y = r_earth * math.cos(lat_rad) * d_lon
        return x, y

    def offboard_heartbeat(self):
        if not self.is_offboard:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9

        if math.isnan(self.lidar_distance) or self.lidar_distance < 3.0:
            self.get_logger().warn(f"EMERGENCY: Lidar Obstacle tripped at {self.lidar_distance}m! Loitering mode engaged.")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LOITER)
            if self.state == 'GLIDE':
                self.state = 'PIVOT'

        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        dx = self.target_ned_x - self.current_x
        dy = self.target_ned_y - self.current_y
        distance_to_target = math.sqrt(dx**2 + dy**2)
        
        target_yaw = math.atan2(dy, dx)
        yaw_error = target_yaw - self.current_heading
        
        while yaw_error > math.pi: yaw_error -= 2 * math.pi
        while yaw_error < -math.pi: yaw_error += 2 * math.pi

        if self.state == 'PIVOT':
            msg.position = [float('nan'), float('nan'), -10.0]
            msg.velocity = [0.0, 0.0, 0.0]
            
            if abs(yaw_error) > 0.15: 
                msg.yawspeed = 0.5 if yaw_error > 0 else -0.5
                msg.yaw = float('nan')
            else:
                self.state = 'GLIDE'
                self.get_logger().info("Vectors Aligned. Engaging 2.0m/s GLIDE thrust.")

        elif self.state == 'GLIDE':
            if abs(yaw_error) > 0.5: 
                self.state = 'PIVOT'
            elif distance_to_target < 2.0:
                self.state = 'LANDING'
                self.get_logger().info(f"Waypoint Reached! Dispatching Auto-Landing block {self.current_wp_index}...")
            else:
                msg.position = [float('nan'), float('nan'), -10.0]
                msg.velocity = [2.0 * math.cos(self.current_heading), 2.0 * math.sin(self.current_heading), 0.0]
                msg.yaw = target_yaw

        elif self.state == 'LANDING':
            # Rely strictly on firmware hardware drop since this simulates the old logic behavior.
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.state = 'AWAITING_TAKEOFF'
            self.landing_timestamp = current_time
            self.get_logger().info("Firmware NAV_LAND engaged. Delaying 10s for package transfer simulation.")
            
        elif self.state == 'AWAITING_TAKEOFF':
            if current_time - self.landing_timestamp > 10.0:
                self.get_logger().info("Timeout threshold hit. Re-engaging next logic cycle.")
                self.current_wp_index += 1
                self.advance_waypoint()

        self.setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = p1
        msg.param2 = p2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SkyPalMissionCommander()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
