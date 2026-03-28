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
        self.proceed_sub = self.create_subscription(String, '/skypal/local_mission_proceed', self.proceed_cb, 10)
        self.status_pub = self.create_publisher(String, '/skypal/mission_status', 10)

        self.get_logger().info("🛡️ Drone Autonomy Core offline and eagerly awaiting UI map dispatch.")

        self.state = 'IDLE' # IDLE -> PIVOT -> GLIDE -> LANDING -> AWAITING_APPROVAL -> (IDLE)
        self.waypoints = []
        self.current_wp_index = 0
        self.is_offboard = False

        self.home_lat = None
        self.home_lon = None
        self.home_alt = None
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_alt = 0.0
        self.current_heading = 0.0
        
        self.alt_history = []
        self.is_physically_landed = False

        self.lidar_distance = 12.0
        self.local_2d_map = {}
        self.breadcrumb_trail = []
        self.scan_original_yaw = 0.0
        self.sweep_yaw = 0.0
        self.dodge_target_yaw = 0.0
        self.dodge_start_x = 0.0
        self.dodge_start_y = 0.0
        self.hop_distance = 6.0
        self.is_grounded = False

        self.target_ned_x = 0.0
        self.target_ned_y = 0.0
        
        self.timer = self.create_timer(0.1, self.offboard_heartbeat)

    def local_mission_cb(self, msg):
        if self.state != 'IDLE':
            self.get_logger().warn("Mission physically in progress! Ignoring local dispatch until current queue clears.")
            return

        coords = msg.data.split(',')
        if len(coords) == 4:
            self.get_logger().info(f"Interlocked GUI Waypoints: {coords} -> Deploying Route Calculator!")
            
            sender_wgs = (float(coords[0]), float(coords[1]))
            receiver_wgs = (float(coords[2]), float(coords[3]))
            home_wgs = (self.home_lat, self.home_lon)
            
            self.waypoints = []
            
            # Segment 1: Home to Sender
            seg1 = self.generate_segment(home_wgs, sender_wgs, 10.0)
            self.waypoints.extend(seg1)
            sx, sy = self.wgs84_to_ned(sender_wgs[0], sender_wgs[1])
            self.waypoints.append({'pos': (sx, sy), 'action': 'LAND', 'tag': 'SENDER'})
            
            # Segment 2: Sender to Receiver
            seg2 = self.generate_segment(sender_wgs, receiver_wgs, 10.0)
            self.waypoints.extend(seg2)
            rx, ry = self.wgs84_to_ned(receiver_wgs[0], receiver_wgs[1])
            self.waypoints.append({'pos': (rx, ry), 'action': 'LAND', 'tag': 'RECEIVER'})
            
            # Segment 3: Receiver to Home
            seg3 = self.generate_segment(receiver_wgs, home_wgs, 10.0)
            self.waypoints.extend(seg3)
            hx, hy = self.wgs84_to_ned(home_wgs[0], home_wgs[1])
            self.waypoints.append({'pos': (hx, hy), 'action': 'LAND', 'tag': 'HOME'})
            
            self.current_wp_index = 0
            self.advance_waypoint()

    def generate_segment(self, start_wgs, end_wgs, interval_m):
        sx, sy = self.wgs84_to_ned(start_wgs[0], start_wgs[1])
        ex, ey = self.wgs84_to_ned(end_wgs[0], end_wgs[1])
        
        dist = math.sqrt((ex-sx)**2 + (ey-sy)**2)
        num_points = max(1, int(dist / interval_m))
        
        points = []
        for i in range(1, num_points):
            frac = i / float(num_points)
            ix = sx + frac * (ex - sx)
            iy = sy + frac * (ey - sy)
            points.append({'pos': (ix, iy), 'action': 'FLY', 'tag': f'ROUTE_BLOCK'})
        return points

    def proceed_cb(self, msg):
        if msg.data == "PROCEED" and self.state == 'AWAITING_APPROVAL':
            self.get_logger().info("✅ Manual UI Proceed Signal Authorized. Launching to next waypoint.")
            self.current_wp_index += 1
            self.advance_waypoint()

    def advance_waypoint(self):
        if hasattr(self, 'landing_z_setpoint'):
            delattr(self, 'landing_z_setpoint')
            
        if self.current_wp_index >= len(self.waypoints):
            self.state = 'IDLE'
            self.get_logger().info("🎉 Complete Mission/Route mapping finished!")
            return
            
        wp = self.waypoints[self.current_wp_index]
        self.target_ned_x = float(wp['pos'][0])
        self.target_ned_y = float(wp['pos'][1])
        self.current_wp_action = str(wp['action'])
        self.current_wp_tag = str(wp['tag'])
        
        if self.state == 'IDLE' or self.state == 'AWAITING_APPROVAL':
            # Only full Pivot on major launches to orient cleanly before accelerating
            self.get_logger().info(f"Targeting explicit route marker: {self.current_wp_tag}")
            self.state = 'PIVOT'
        else:
            # Intermediate checkpoints flow instantaneously without pausing
            self.state = 'GLIDE'      # Fire signal to multiplexer to unblock channel
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
        self.lidar_distance = msg.current_distance

    def global_pos_cb(self, msg):
        self.current_lat = msg.lat
        self.current_lon = msg.lon
        self.current_alt = msg.alt
        
        # Custom mathematical hardware touchdown verification buffer
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.alt_history.append((current_time, msg.alt))
        self.alt_history = [x for x in self.alt_history if current_time - x[0] <= 2.0]
        
        if len(self.alt_history) >= 5:
            alts = [x[1] for x in self.alt_history]
            if max(alts) - min(alts) < 0.15:
                # If variance strictly falls under 15cm over 2000 milliseconds, it is definitively blocked by concrete
                self.is_physically_landed = True
            else:
                self.is_physically_landed = False
        
        if self.home_lat is None and msg.lat != 0.0:
            self.home_lat = msg.lat
            self.home_lon = msg.lon
            self.home_alt = msg.alt
            self.get_logger().info(f"📍 Home Base Station physically Locked: {self.home_lat}, {self.home_lon} at {self.home_alt:.2f}m")

    def attitude_cb(self, msg):
        # Decode Quaternion mathematically to extract Yaw (Heading)
        q = msg.q
        self.current_heading = math.atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]))

    def wgs84_to_ned(self, target_lat, target_lon):
        if self.home_lat is None or self.home_lon is None:
            return 0.0, 0.0
            
        r_earth = 6371000.0  # Radius of earth in meters
        
        home_lat_rad = math.radians(float(self.home_lat))
        home_lon_rad = math.radians(float(self.home_lon))
        target_lat_rad = math.radians(target_lat)
        target_lon_rad = math.radians(target_lon)
        
        d_lat = target_lat_rad - home_lat_rad
        d_lon = target_lon_rad - home_lon_rad
        
        x = r_earth * d_lat
        y = r_earth * math.cos(home_lat_rad) * d_lon
        return x, y

    def offboard_heartbeat(self):
        if not self.is_offboard:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9

        # Blind the LiDAR collision protocol specifically when launching from or touching the ground
        is_near_ground = False
        if self.home_alt is not None:
            if abs(self.current_alt - float(self.home_alt)) < 4.0:
                is_near_ground = True

        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # Dynamic WGS84 offset mapping bypassing isolated local position channels
        current_ned_x, current_ned_y = self.wgs84_to_ned(self.current_lat, self.current_lon)
        dx = self.target_ned_x - current_ned_x
        dy = self.target_ned_y - current_ned_y
        distance_to_target = math.sqrt(dx**2 + dy**2)
        
        if distance_to_target > 1.0:
            target_yaw = math.atan2(dy, dx)
        else:
            target_yaw = self.current_heading
            
        yaw_error = target_yaw - self.current_heading
        
        while yaw_error > math.pi: yaw_error -= 2 * math.pi
        while yaw_error < -math.pi: yaw_error += 2 * math.pi

        if self.state == 'PIVOT':
            # Stop moving horizontally, just spin the yaw and climb to Z=-10.0
            vx = 0.0
            vy = 0.0
            
            current_ned_z = -(self.current_alt - float(self.home_alt)) if self.home_alt is not None else 0.0
            dz = -10.0 - current_ned_z
            vz = max(-2.0, min(2.0, dz * 0.5))
            
            msg.position = [float('nan'), float('nan'), float('nan')]
            msg.velocity = [vx, vy, vz]
            
            if abs(yaw_error) > 0.15: 
                msg.yawspeed = 0.5 if yaw_error > 0 else -0.5
                msg.yaw = float('nan')
            elif abs(dz) < 0.5:
                self.state = 'GLIDE'
                self.get_logger().info("Altitude Reached! Engaging 3.0m/s GLIDE thrust.")

        elif self.state == 'GLIDE':
            current_ned_z = -(self.current_alt - float(self.home_alt)) if self.home_alt is not None else 0.0
            
            # Autonomous Architecture Obstacle Intercept Trigger
            if abs(current_ned_z) >= 4.0 and self.lidar_distance < 4.0 and self.current_wp_action != 'LAND':
                self.get_logger().warn(f"OBSTACLE DETECTED at {self.lidar_distance:.1f}m! Initiating 180° Grid-Hop Radar Scan.")
                self.scan_original_yaw = self.current_heading
                self.sweep_yaw = self.current_heading
                self.local_2d_map = {}
                self.state = 'EVADE_SCAN_LEFT'
                return
                
            # Dynamic thresholding logic: fly-through nodes (FLY) span loosely to preserve momentum, terminal nodes (LAND) tighten to 0.3m.
            threshold = 1.5 if self.current_wp_action == 'FLY' else 0.3
            
            if distance_to_target < threshold:
                if self.current_wp_action == 'LAND':
                    self.state = 'LANDING'
                    self.get_logger().info(f"Terminus Reached! Dispatching Parabolic Auto-Landing {self.current_wp_tag}...")
                    self.landing_start_time = self.get_clock().now().nanoseconds / 1e9
                else:
                    self.get_logger().info("Crossed discrete routing block, mapping next sequence natively...")
                    self.current_wp_index += 1
                    self.advance_waypoint()
            else:
                if self.current_wp_action == 'FLY':
                    # Cruise cleanly through trace waypoints
                    Kp = 3.0
                    vx = (dx / distance_to_target) * Kp if distance_to_target > 0.1 else 0.0
                    vy = (dy / distance_to_target) * Kp if distance_to_target > 0.1 else 0.0
                else:
                    # Precise Proportional Geometry strictly targeting the payload marker, decaying thrust identically to the error
                    Kp = 1.0
                    vx = dx * Kp
                    vy = dy * Kp
                    speed = math.sqrt(vx**2 + vy**2)
                    if speed > 3.0:
                        vx = (vx / speed) * 3.0
                        vy = (vy / speed) * 3.0
                
                current_ned_z = -(self.current_alt - float(self.home_alt)) if self.home_alt is not None else 0.0
                dz = -10.0 - current_ned_z
                vz = max(-2.0, min(2.0, dz * 0.5))
                
                msg.position = [float('nan'), float('nan'), float('nan')]
                msg.velocity = [vx, vy, vz]
                
                msg.yaw = target_yaw

        elif self.state == 'EVADE_SCAN_LEFT':
            target_left = self.scan_original_yaw - 1.5708
            while target_left < -math.pi: target_left += 2.0 * math.pi
            err = abs(target_left - self.current_heading)
            if err > math.pi: err = 2.0 * math.pi - err
            
            if err < 0.15:
                 self.get_logger().info("Oriented to Left Bound (-90°). Commencing High-Density 180° Right Sweep!")
                 self.sweep_yaw = self.current_heading
                 self.state = 'EVADE_SCAN_RIGHT'
            else:
                 self.sweep_yaw = self.current_heading - 0.04
                 while self.sweep_yaw < -math.pi: self.sweep_yaw += 2.0 * math.pi
                 
                 msg.position = [float('nan'), float('nan'), float('nan')]
                 current_ned_z = -(self.current_alt - float(self.home_alt)) if self.home_alt is not None else 0.0
                 msg.velocity = [0.0, 0.0, max(-2.0, min(2.0, (-10.0 - current_ned_z) * 0.5))]
                 msg.yaw = self.sweep_yaw
                 
                 # Depth Mapping Array Tracker
                 rel_yaw = self.current_heading - self.scan_original_yaw
                 while rel_yaw > math.pi: rel_yaw -= 2.0 * math.pi
                 while rel_yaw < -math.pi: rel_yaw += 2.0 * math.pi
                 
                 bin_idx = int(math.degrees(rel_yaw) / 5.0) * 5
                 dist = self.lidar_distance if not math.isnan(self.lidar_distance) else 12.0
                 if bin_idx not in self.local_2d_map or dist < self.local_2d_map[bin_idx]:
                     self.local_2d_map[bin_idx] = dist
                     
        elif self.state == 'EVADE_SCAN_RIGHT':
            target_right = self.scan_original_yaw + 1.5708
            while target_right > math.pi: target_right -= 2.0 * math.pi
            err = abs(target_right - self.current_heading)
            if err > math.pi: err = 2.0 * math.pi - err
            
            if err < 0.15:
                 self.get_logger().info(f"180° Sweep Complete! Captured {len(self.local_2d_map)} spatial footprint blocks. Analyzing clearance geometry...")
                 self.state = 'EVADE_COMPUTE'
            else:
                 self.sweep_yaw = self.current_heading + 0.04
                 while self.sweep_yaw > math.pi: self.sweep_yaw -= 2.0 * math.pi
                 
                 msg.position = [float('nan'), float('nan'), float('nan')]
                 current_ned_z = -(self.current_alt - float(self.home_alt)) if self.home_alt is not None else 0.0
                 msg.velocity = [0.0, 0.0, max(-2.0, min(2.0, (-10.0 - current_ned_z) * 0.5))]
                 msg.yaw = self.sweep_yaw
                 
                 # Depth Mapping Array Tracker
                 rel_yaw = self.current_heading - self.scan_original_yaw
                 while rel_yaw > math.pi: rel_yaw -= 2.0 * math.pi
                 while rel_yaw < -math.pi: rel_yaw += 2.0 * math.pi
                 
                 bin_idx = int(math.degrees(rel_yaw) / 5.0) * 5
                 dist = self.lidar_distance if not math.isnan(self.lidar_distance) else 12.0
                 if bin_idx not in self.local_2d_map or dist < self.local_2d_map[bin_idx]:
                     self.local_2d_map[bin_idx] = dist

        elif self.state == 'EVADE_COMPUTE':
            safe_bins = sorted([b for b, dist in self.local_2d_map.items() if dist >= 6.0])
            clusters = []
            curr_cluster = [safe_bins[0]] if safe_bins else []
            for i in range(1, len(safe_bins)):
                if safe_bins[i] - safe_bins[i-1] <= 35:
                    curr_cluster.append(safe_bins[i])
                else:
                    clusters.append(curr_cluster)
                    curr_cluster = [safe_bins[i]]
            if curr_cluster: clusters.append(curr_cluster)
            
            valid_clusters = []
            for c in clusters:
                if len(c) == 0: continue
                angular_width_deg = abs(c[-1] - c[0]) + 5.0
                physical_width_m = 6.0 * math.radians(angular_width_deg) 
                
                if physical_width_m >= 1.0: 
                    # Breadcrumb filter block
                    center_bin = sum(c) / len(c)
                    candidate_yaw = self.scan_original_yaw + math.radians(center_bin)
                    
                    hop_dist = min(6.0, distance_to_target)
                    hyp_x = current_ned_x + hop_dist * math.cos(candidate_yaw)
                    hyp_y = current_ned_y + hop_dist * math.sin(candidate_yaw)
                    
                    too_close = False
                    for crumb in self.breadcrumb_trail:
                        dist_to_crumb = math.sqrt((hyp_x - crumb[0])**2 + (hyp_y - crumb[1])**2)
                        if hop_dist > 3.5 and dist_to_crumb <= 3.5:
                            too_close = True
                            self.get_logger().warn(f"Tabu Filter: Rejecting Gap at {center_bin:.1f}°. Tracks into historically blocked array!")
                            break
                    if not too_close:
                        valid_clusters.append(c)
                else:
                    self.get_logger().warn(f"Physics Geometry Filter: Rejecting physical gap width of {physical_width_m:.1f}m. Smaller than frame clearance.")
                    
            if valid_clusters:
                 best_cluster = min(valid_clusters, key=lambda c: abs(sum(c)/len(c)))
                 center_bin = sum(best_cluster) / len(best_cluster)
                 self.dodge_target_yaw = self.scan_original_yaw + math.radians(center_bin)
                 self.get_logger().info(f"Target Line Obscured. Diverting Grid-Hop Vector smoothly through {center_bin:.1f}°")
            elif clusters:
                 best_cluster = min(clusters, key=lambda c: abs(sum(c)/len(c)))
                 center_bin = sum(best_cluster) / len(best_cluster)
                 self.dodge_target_yaw = self.scan_original_yaw + math.radians(center_bin)
                 self.get_logger().warn(f"Geometry Width requirements failed. Squeezing tightly through fragmented {center_bin:.1f}° target.")
            else:
                 self.get_logger().error("FATAL: Map is mathematically completely horizontally bound! Punching directly over coordinates safely!")
                 self.dodge_target_yaw = self.scan_original_yaw
                 
            self.dodge_start_x = current_ned_x
            self.dodge_start_y = current_ned_y
            self.hop_distance = min(6.0, distance_to_target)
            self.state = 'EVADE_HOP'

        elif self.state == 'EVADE_HOP':
            dist_flown = math.sqrt((current_ned_x - self.dodge_start_x)**2 + (current_ned_y - self.dodge_start_y)**2)
            
            if dist_flown >= self.hop_distance or distance_to_target < 1.0:
                self.get_logger().info(f"{self.hop_distance:.1f}m Sprint Hop complete. Re-acquiring target trajectory into primary GLIDE trace.")
                self.breadcrumb_trail.append((current_ned_x, current_ned_y))
                if len(self.breadcrumb_trail) > 20: self.breadcrumb_trail.pop(0)
                self.state = 'GLIDE'
                return
                
            msg.position = [float('nan'), float('nan'), float('nan')]
            vx = 2.5 * math.cos(self.dodge_target_yaw)
            vy = 2.5 * math.sin(self.dodge_target_yaw)
            
            current_ned_z = -(self.current_alt - float(self.home_alt)) if self.home_alt is not None else 0.0
            msg.velocity = [vx, vy, max(-2.0, min(2.0, (-10.0 - current_ned_z) * 0.5))]
            msg.yaw = self.dodge_target_yaw

        elif self.state == 'LANDING':
            # Target is already enveloped within 0.5m variance. Arrest all horizontal P-loops instantly to kill oscillations.
            vx = 0.0
            vy = 0.0
                
            # Execute 2-Stage Parabolic Velocity Drop: 1.0 m/s free-fall until 2.0m, then 0.15 m/s buttery cushion
            current_ned_z = -(self.current_alt - float(self.home_alt)) if self.home_alt is not None else 0.0
            if current_ned_z < -2.0:
                vz = 1.0
            else:
                vz = 0.15
            
            if int(current_time * 10) % 20 == 0:
                self.get_logger().info(f"Descending precisely via explicit velocity... Z-Plane at: {current_ned_z:.2f}m")
            
            msg.position = [float('nan'), float('nan'), float('nan')]
            msg.velocity = [vx, vy, vz]
            msg.yaw = target_yaw
            
            # Since PX4 rejects standard disarms while actively in Offboard without local bounds, we deploy the 21196 hardware MAGIC_FORCE_KILL override
            # We strictly enforce current_ned_z > -1.0 to guarantee the mathematical variance buffer doesn't trip a false positive at the -10.0m layer.
            if current_time - self.landing_start_time > 2.0 and self.is_physically_landed and current_ned_z > -1.0:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)
                self.state = 'AWAITING_APPROVAL'
                
                # Push GUI Checkpoint Notification — prefix with ARRIVED_ so the GUI listener matches
                status_msg = String()
                status_msg.data = f"ARRIVED_{self.current_wp_tag}"
                self.status_pub.publish(status_msg)
                
                self.get_logger().info(f"Touchdown confirmed! GUI notified with status: {status_msg.data}")
            
        elif self.state == 'AWAITING_APPROVAL':
            # Publish explicit zero-velocity grounding arrays while dropping the Position block completely
            msg.position = [float('nan'), float('nan'), float('nan')]
            msg.velocity = [0.0, 0.0, 0.0]
            msg.yaw = target_yaw

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
