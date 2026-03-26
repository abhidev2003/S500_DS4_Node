#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, qos_profile_sensor_data
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, VehicleGlobalPosition, VehicleAttitude, DistanceSensor, VehicleLandDetected
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
        self.sys_pub = self.create_publisher(String, '/skypal/sys_command', 10)

        # Utilize Sensor Data QoS strictly to combat FastDDS strict matching blockades
        self.global_pos_sub = self.create_subscription(VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.global_pos_cb, qos_profile_sensor_data)
        self.attitude_sub = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_cb, qos_profile_sensor_data)
        self.distance_sub = self.create_subscription(DistanceSensor, '/fmu/out/distance_sensor', self.distance_cb, qos_profile)
        self.land_sub = self.create_subscription(VehicleLandDetected, '/fmu/out/vehicle_land_detected', self.land_cb, qos_profile_sensor_data)
        
        self.local_mission_sub = self.create_subscription(String, '/skypal/local_mission', self.local_mission_cb, 10)

        self.get_logger().info("🛡️ Drone Autonomy Core offline and eagerly awaiting UI map dispatch.")

        self.state = 'IDLE' # IDLE -> PIVOT -> GLIDE -> LANDING -> AWAITING_TAKEOFF -> (IDLE)
        self.waypoints = []
        self.current_wp_index = 0
        self.landing_timestamp = 0.0
        self.is_offboard = False

        self.home_lat = None
        self.home_lon = None
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_heading = 0.0
        
        self.lidar_distance = float('inf')
        self.lidar_active = False
        self.is_grounded = False

        self.target_ned_x = 0.0
        self.target_ned_y = 0.0
        
        self.timer = self.create_timer(0.1, self.offboard_heartbeat)

    def local_mission_cb(self, msg):
        if self.state != 'IDLE':
            self.get_logger().warn("Mission physically in progress! Ignoring local dispatch until current queue clears.")
            return
            
        parts = [float(x) for x in msg.data.split(',')]
        if len(parts) == 4 and self.home_lat is not None:
            self.get_logger().info("📥 Local Multi-Waypoint Mission Intercepted via GUI Map!")
            send_lat, send_lon, recv_lat, recv_lon = parts
            
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
        
        # Fire signal to multiplexer to unblock channel
        sys_msg = String()
        sys_msg.data = "auto_mode_yield"
        self.sys_pub.publish(sys_msg)
        
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0) # Offboard
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.is_offboard = True
        self.state = 'PIVOT'

    def land_cb(self, msg):
        self.is_grounded = msg.ground_contact or msg.landed

    def distance_cb(self, msg):
        if msg.orientation == 1 or True: 
            self.lidar_distance = msg.current_distance
            self.lidar_active = True

    def global_pos_cb(self, msg):
        self.current_lat = msg.lat
        self.current_lon = msg.lon
        
        if self.home_lat is None and msg.lat != 0.0:
            self.home_lat = msg.lat
            self.home_lon = msg.lon
            self.get_logger().info(f"📍 Home Base Station physically Locked: {self.home_lat}, {self.home_lon}")

    def attitude_cb(self, msg):
        # Decode Quaternion mathematically to extract Yaw (Heading)
        q = msg.q
        self.current_heading = math.atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]))

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

        # Dynamic WGS84 offset mapping bypassing isolated local position channels
        current_ned_x, current_ned_y = self.wgs84_to_ned(self.current_lat, self.current_lon)
        dx = self.target_ned_x - current_ned_x
        dy = self.target_ned_y - current_ned_y
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
            # Abandon native NAV_LAND. Enforce physical buttery-smooth cushioned 0.15m/s descent vectors precisely identically to RTL
            msg.position = [float('nan'), float('nan'), float('nan')]
            msg.velocity = [0.0, 0.0, 0.15]
            msg.yaw = target_yaw
            
            # Instantly kill propellers upon authentic pavement contact to prevent 0.2m freefall
            if self.is_grounded:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
                self.state = 'AWAITING_TAKEOFF'
                self.landing_timestamp = current_time
                self.get_logger().info("Touchdown verified natively by PX4 Accelerometer! Disarmed. Delaying 10s for package transfer simulation.")
            
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
