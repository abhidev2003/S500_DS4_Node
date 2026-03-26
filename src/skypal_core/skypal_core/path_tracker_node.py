#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleGlobalPosition, TrajectorySetpoint, VehicleCommand, VehicleLandDetected
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import math
import time

class PathTrackerNode(Node):
    def __init__(self):
        super().__init__('path_tracker_node')
        
        from rclpy.qos import qos_profile_sensor_data
        
        self.global_pos_sub = self.create_subscription(VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.pos_callback, qos_profile_sensor_data)
        self.sys_cmd_sub = self.create_subscription(String, '/skypal/sys_command', self.sys_cmd_callback, 10)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, '/skypal/autonomous_trajectory', 10)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.land_sub = self.create_subscription(VehicleLandDetected, '/fmu/out/vehicle_land_detected', self.land_cb, qos_profile_sensor_data)
        
        self.path = [] # List of (lat, lon, alt)
        self.record_threshold = 1.0 # Meters logic
        self.is_rtl = False
        self.target_index = -1
        self.is_grounded = False
        
        self.alt_history = []
        self.is_physically_landed = False
        self.rtl_land_start_time = 0.0
        
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_alt = 0.0
        
        self.dt = 0.05 
        self.timer = self.create_timer(self.dt, self.execute_rtl)
        
        self.get_logger().info("Skypal Global Path Tracker Started. Intercepting GPS...")

    def land_cb(self, msg):
        self.is_grounded = msg.ground_contact or msg.landed

    def wgs84_to_ned(self, target_lat, target_lon, ref_lat, ref_lon):
        r_earth = 6378137.0
        lat_rad = math.radians(ref_lat)
        target_lat_rad = math.radians(target_lat)
        
        d_lat = target_lat_rad - lat_rad
        d_lon = math.radians(target_lon) - math.radians(ref_lon)
        
        x = r_earth * d_lat
        y = r_earth * math.cos(lat_rad) * d_lon
        return x, y

    def pos_callback(self, msg):
        # We must ignore the origin null island lock
        if msg.lat == 0.0 and msg.lon == 0.0:
            return
            
        self.current_lat = msg.lat
        self.current_lon = msg.lon
        self.current_alt = msg.alt
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.alt_history.append((current_time, msg.alt))
        self.alt_history = [x for x in self.alt_history if current_time - x[0] <= 2.0]
        
        if len(self.alt_history) >= 5:
            alts = [x[1] for x in self.alt_history]
            if max(alts) - min(alts) < 0.15:
                self.is_physically_landed = True
            else:
                self.is_physically_landed = False
        
        if not self.is_rtl:
            if len(self.path) == 0:
                self.path.append((msg.lat, msg.lon, msg.alt))
                self.get_logger().info("First Global Origin locked. Tracking outbound sequence...")
            else:
                last_lat, last_lon, last_alt = self.path[-1]
                dx, dy = self.wgs84_to_ned(msg.lat, msg.lon, last_lat, last_lon)
                dz_ned = -(msg.alt - last_alt)
                
                dist = math.sqrt(dx**2 + dy**2 + dz_ned**2)
                if dist > self.record_threshold:
                    self.path.append((msg.lat, msg.lon, msg.alt))
        
    def sys_cmd_callback(self, msg):
        if msg.data == "nav_rtl":
            if not self.is_rtl:
                self.get_logger().info(f"RTL Triggered! Engaged Trace-Back. Route length: {len(self.path)} steps.")
                self.is_rtl = True
                if len(self.path) > 0:
                    self.target_index = len(self.path) - 1

    def execute_rtl(self):
        if not self.is_rtl:
            return
            
        if self.target_index < 0:
            # Reached Home Origin. Execute a heavily cushioned landing.
            if not hasattr(self, 'landing_z_setpoint'):
                if len(self.path) > 0:
                    self.landing_z_setpoint = -(self.current_alt - self.path[0][2])
                else:
                    self.landing_z_setpoint = -10.0
            
            # The heart_node multiplexer currently forces `OffboardControlMode.position = True`.
            # To prevent PX4 from overriding our descent logic by snapping back to its last known altitude lock,
            # we must explicitly command downward Position arrays rather than pure Velocity.
            # 2-Stage Drop Sequence: 1.0 m/s fall until 2 meters, then 0.15 m/s terminal cushion. (dt=0.05s)
            if self.landing_z_setpoint < -2.0:
                self.landing_z_setpoint += 0.05
            else:
                self.landing_z_setpoint += 0.0075

            setpoint = TrajectorySetpoint()
            setpoint.position = [0.0, 0.0, self.landing_z_setpoint] # 0,0 NED perfectly anchors the drone over the exact physical launch footprint
            setpoint.velocity = [0.0, 0.0, 0.0]
            setpoint.yawspeed = 0.0
            setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.traj_pub.publish(setpoint)
            
            if self.rtl_land_start_time == 0.0:
                self.rtl_land_start_time = self.get_clock().now().nanoseconds / 1e9
            
            current_time = self.get_clock().now().nanoseconds / 1e9
            if current_time - self.rtl_land_start_time > 2.0 and self.is_physically_landed:
                msg = VehicleCommand()
                msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
                msg.param1 = 0.0 # Disarm payload
                msg.param2 = 21196.0 # MAGIC FORCE KILL override
                msg.target_system = 1
                msg.target_component = 1
                msg.source_system = 1
                msg.source_component = 1
                msg.from_external = True
                msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                self.command_pub.publish(msg)
                self.get_logger().info("Raw Ground Lock Detected. Track sequence finished.")
            return
            
        target_lat, target_lon, target_alt = self.path[self.target_index]
        dx, dy = self.wgs84_to_ned(target_lat, target_lon, self.current_lat, self.current_lon)
        dz_ned = -(target_alt - self.current_alt) 
        
        dist = math.sqrt(dx**2 + dy**2 + dz_ned**2)
        
        if dist < 1.0: 
            self.target_index -= 1
            if self.target_index < 0:
                self.get_logger().info("Successfully arrived at Home Origin. Holding hover.")
                return
            target_lat, target_lon, target_alt = self.path[self.target_index]
            dx, dy = self.wgs84_to_ned(target_lat, target_lon, self.current_lat, self.current_lon)
            dz_ned = -(target_alt - self.current_alt)
            dist = math.sqrt(dx**2 + dy**2 + dz_ned**2)

        fly_speed = 3.0 
        if dist > 0:
            vx = (dx / dist) * fly_speed
            vy = (dy / dist) * fly_speed
            vz = (dz_ned / dist) * fly_speed
        else:
            vx, vy, vz = 0.0, 0.0, 0.0
            
        yaw = math.atan2(vy, vx)
        
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
