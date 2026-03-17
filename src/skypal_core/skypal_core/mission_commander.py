#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleGlobalPosition, VehicleLocalPosition, DistanceSensor
import firebase_admin
from firebase_admin import credentials, firestore
import math
import numpy as np
import os

class SkyPalMissionCommander(Node):
    def __init__(self):
        super().__init__('skypal_mission_commander')

        # 1. PX4 MicroDDS Publishers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # 2. GPS & Local Position Subscribers
        self.global_pos_sub = self.create_subscription(VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.global_pos_cb, 10)
        self.local_pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_pos_cb, 10)
        self.distance_sub = self.create_subscription(DistanceSensor, '/fmu/out/distance_sensor', self.distance_cb, qos_profile)

        # 3. Firebase Initialization
        self.db = None
        key_path = "serviceAccountKey.json"
        
        if os.path.exists(key_path):
            try:
                cred = credentials.Certificate(key_path)
                firebase_admin.initialize_app(cred)
                self.db = firestore.client()
                self.get_logger().info("🔥 Firebase Admin SDK Authenticated Successfully.")
            except Exception as e:
                self.get_logger().error(f"Firebase Initialization Error: {e}")
        else:
            self.get_logger().warn(f"WARN: {key_path} not found. Running in dry-run mode without Cloud connection.")

        # 4. Mission State Variables
        self.mission_data = None
        self.is_offboard = False
        self.doc_ref = None
        self.state = 'IDLE'

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
        
        self.last_upload_time = 0.0
        
        # Start the Cloud Listener if db exists
        if self.db:
            self.init_firebase_listener()
        
        # High-frequency timer for PX4 Offboard (10Hz)
        self.timer = self.create_timer(0.1, self.offboard_heartbeat)

    def distance_cb(self, msg):
        # Read forward Lunar Lidar (assuming orientation is forward)
        # 1 stands for front-facing distance sensor
        if msg.orientation == 1 or True: # Assuming first distance sensor is front Lidar
            self.lidar_distance = msg.current_distance
            self.lidar_active = True

    def global_pos_cb(self, msg):
        self.current_lat = msg.lat
        self.current_lon = msg.lon
        
        # Lock in the first valid GPS reading as Home for Offset Calculations
        if self.home_lat is None and msg.lat != 0.0:
            self.home_lat = msg.lat
            self.home_lon = msg.lon
            self.get_logger().info(f"📍 Home GPS Locked at: {self.home_lat}, {self.home_lon}")

        # Update Firebase liveLocation precisely at ~1Hz to save bandwidth
        current_time = self.get_clock().now().nanoseconds / 1e9
        if self.doc_ref and (current_time - self.last_upload_time > 1.0):
            try:
                self.doc_ref.update({'liveLocation': {'lat': msg.lat, 'lng': msg.lon}})
                self.last_upload_time = current_time
            except Exception as e:
                pass # Suppress minor network dropout warnings

    def local_pos_cb(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        self.current_heading = msg.heading

    def init_firebase_listener(self):
        # Listen for any new mission Arjun sends via React Native App
        query = self.db.collection('landing_missions').where('status', '==', 'pending').limit(1)
        query.on_snapshot(self.on_snapshot)

    def on_snapshot(self, doc_snapshot, changes, read_time):
        for doc in doc_snapshot:
            self.mission_data = doc.to_dict()
            self.doc_ref = doc.reference
            self.get_logger().info(f"🚀 Mission Triggered by App: {doc.id}")
            self.start_sequence()

    def wgs84_to_ned(self, target_lat, target_lon):
        # Earth radius in meters
        r_earth = 6378137.0
        
        lat_rad = math.radians(self.home_lat)
        lon_rad = math.radians(self.home_lon)
        target_lat_rad = math.radians(target_lat)
        target_lon_rad = math.radians(target_lon)
        
        d_lat = target_lat_rad - lat_rad
        d_lon = target_lon_rad - lon_rad
        
        # NED mapping: North=X, East=Y
        x = r_earth * d_lat
        y = r_earth * math.cos(lat_rad) * d_lon
        return x, y

    def start_sequence(self):
        if not self.home_lat:
            self.get_logger().error("ABORT: Cannot initiate mission without valid GPS Lock!")
            return

        # Extract WGS84 Coords from Firebase Document
        target_lat = self.mission_data.get('receiverCoords', {}).get('lat', self.home_lat)
        target_lon = self.mission_data.get('receiverCoords', {}).get('lng', self.home_lon)
        
        # Translate to Local NED
        self.target_ned_x, self.target_ned_y = self.wgs84_to_ned(target_lat, target_lon)
        self.get_logger().info(f"Target Acquired in NED frame: X={self.target_ned_x:.2f}m, Y={self.target_ned_y:.2f}m")

        # Notify the Mobile App we are moving
        self.doc_ref.update({'status': 'active'})
        
        # Engage Offboard Mode and Arm Propellers
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0) # 6.0 is Offboard
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        
        self.is_offboard = True
        self.state = 'PIVOT'

    def offboard_heartbeat(self):
        # Failsafe: PX4 will crash if TrajectorySetpoint is not received > 500ms
        if not self.is_offboard:
            return

        # Lunar Lidar Emergency Failsafe (Checks NaN or <3m)
        if math.isnan(self.lidar_distance) or self.lidar_distance < 3.0:
            self.get_logger().warn(f"EMERGENCY: Lidar Obstacle at {self.lidar_distance}m! Loitering.")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LOITER)
            # Revert to PIVOT state to look for clear path
            if self.state == 'GLIDE':
                self.state = 'PIVOT'

        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        dx = self.target_ned_x - self.current_x
        dy = self.target_ned_y - self.current_y
        distance_to_target = math.sqrt(dx**2 + dy**2)
        
        target_yaw = math.atan2(dy, dx)
        yaw_error = target_yaw - self.current_heading
        
        # Normalize yaw error to [-pi, pi]
        while yaw_error > math.pi: yaw_error -= 2 * math.pi
        while yaw_error < -math.pi: yaw_error += 2 * math.pi

        # ----------------------------------------------------
        # THE PIVOT-AND-GLIDE STATE MACHINE
        # ----------------------------------------------------
        if self.state == 'PIVOT':
            # Halt translation, focus on turning nose toward target waypoint
            msg.position = [float('nan'), float('nan'), -10.0]
            msg.velocity = [0.0, 0.0, 0.0]
            
            if abs(yaw_error) > 0.15: # ~8 degrees tolerance
                msg.yawspeed = 0.5 if yaw_error > 0 else -0.5
                msg.yaw = float('nan')
            else:
                self.state = 'GLIDE'
                self.get_logger().info("Aligned. Engaging GLIDE.")

        elif self.state == 'GLIDE':
            # Maintain altitude, drive forward along current heading
            if abs(yaw_error) > 0.5: # Wind knocked us off course
                self.state = 'PIVOT'
            elif distance_to_target < 2.0:
                self.state = 'LAND/HOVER'
                self.get_logger().info("Target Reached. Engaging HOVER for QR Scan.")
            else:
                msg.position = [float('nan'), float('nan'), -10.0]
                # Forward thrust mapping (FRD frame approximated)
                msg.velocity = [2.0 * math.cos(self.current_heading), 2.0 * math.sin(self.current_heading), 0.0]
                msg.yaw = target_yaw

        elif self.state == 'LAND/HOVER':
            # Hold geometric X/Y position at 2m altitude for user to scan QR code
            msg.position = [self.target_ned_x, self.target_ned_y, -2.0]
            msg.yaw = float('nan')
            msg.velocity = [float('nan'), float('nan'), float('nan')]

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
