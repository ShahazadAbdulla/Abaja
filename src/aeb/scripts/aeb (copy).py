#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import time
from enum import Enum

from vehiclecontrol.msg import Control
from radar_msgs.msg import RadarTrackList
from feedback.msg import Velocity
from inertial_msgs.msg import Pose

class AEBState(Enum):
    ACCELERATING = "accelerating"
    CRUISING = "cruising"
    BRAKING = "braking"
    EMERGENCY_STOP = "emergency_stop"
    PERMANENT_STOP = "permanent_stop"  # NEW: Permanent stop state

class ObjectType(Enum):
    UNKNOWN = "unknown"
    VEHICLE = "vehicle"
    PEDESTRIAN = "pedestrian"
    CYCLIST = "cyclist"
    STATIC_OBSTACLE = "static_obstacle"

class AEBController(Node):
    def __init__(self):
        super().__init__('aeb_controller')
        self.get_logger().info("Starting Permanent Stop AEB Controller...")

        # User Requirements
        self.TARGET_SPEED_KMPH = 30.0  # Target speed: 30 km/h
        self.TARGET_SPEED_MS = self.TARGET_SPEED_KMPH / 3.6  # Convert to m/s
        self.ACCELERATION_DISTANCE = 100.0  # Accelerate over 100 meters
        self.BRAKING_DISTANCE_THRESHOLD = 6.0  # Emergency stop at 6 meters
        self.BRAKING_DECELERATION = 8.0  # Brake at 8 m/s²
        self.RADAR_MONITORING_RANGE = 40.0  # Monitor obstacles from 40 meters
        
        # PERMANENT STOP parameters
        self.DANGER_ZONE_MIN = 5.0  # Start danger zone at 4 meters
        self.DANGER_ZONE_MAX = 7.5  # End danger zone at 8 meters
        self.VEHICLE_PATH_WIDTH = 3.0  # Vehicle path width in meters
        self.ANY_OVERLAP_THRESHOLD = 0.1  # ANY overlap triggers response
        self.IN_PATH_THRESHOLD = 1.5  # Object within 1.5m of center is "in front"
        self.SPEED_THRESHOLD_FOR_STOPPED = 0.5  # Consider vehicle stopped below this speed
        
        # Object classification parameters
        self.RCS_VEHICLE_MIN = 10.0
        self.RCS_PEDESTRIAN_MIN = -25.0
        self.RCS_PEDESTRIAN_MAX = 1.9
        self.RCS_CYCLIST_MIN = -5.0
        self.RCS_CYCLIST_MAX = 1.9
        self.SPEED_STATIC_THRESHOLD = 0.5
        self.SPEED_PEDESTRIAN_MAX = 2.5
        self.SPEED_CYCLIST_MAX = 8.0
        
        # State variables
        self.current_speed = 0.0
        self.distance_traveled = 0.0
        self.last_position = None
        self.last_time = None
        self.current_state = AEBState.ACCELERATING
        
        # Obstacle tracking
        self.closest_obstacle_distance = float('inf')
        self.closest_obstacle_type = ObjectType.UNKNOWN
        self.obstacle_in_path = False
        self.overlap_detected = False
        self.overlap_amount = 0.0
        
        # PERMANENT STOP tracking - NEVER auto-clears
        self.permanent_stop_triggered = False
        self.stop_trigger_reason = ""
        self.stop_trigger_distance = 0.0
        self.permanent_stop_timestamp = None
        
        # Control outputs
        self.throttle_output = 0.0
        self.brake_output = 0.0
        self.steering_output = 0.0
        
        # ROS2 Subscriptions and Publishers
        self.radar_sub = self.create_subscription(
            RadarTrackList, 'RadarObjects', self.radar_callback, 10)
        self.velocity_sub = self.create_subscription(
            Velocity, 'VehicleSpeed', self.velocity_callback, 10)
        self.inertial_sub = self.create_subscription(
            Pose, 'InertialData', self.inertial_callback, 10)
        
        # Control publisher
        self.control_pub = self.create_publisher(Control, '/vehicle_control', 10)
        
        # Control loop timer (50Hz = 0.02s)
        self.control_timer = self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info("Permanent Stop AEB Controller initialized successfully")

    def classify_object(self, radar_obj) -> ObjectType:
        """Classify detected object based on RCS and speed characteristics"""
        relative_speed_magnitude = math.sqrt(radar_obj.vx**2 + radar_obj.vy**2)
        rcs_value = getattr(radar_obj, 'rcs', 0.0)
        
        # Vehicle classification (high RCS)
        if rcs_value >= self.RCS_VEHICLE_MIN:
            return ObjectType.VEHICLE
        
        # Static/slow moving object classification
        is_relatively_static = relative_speed_magnitude < self.SPEED_STATIC_THRESHOLD
        if is_relatively_static:
            if self.RCS_PEDESTRIAN_MIN <= rcs_value <= self.RCS_PEDESTRIAN_MAX:
                return ObjectType.PEDESTRIAN
            elif self.RCS_CYCLIST_MIN <= rcs_value <= self.RCS_CYCLIST_MAX:
                return ObjectType.CYCLIST
            else:
                return ObjectType.STATIC_OBSTACLE
        
        # Moving object classification
        if self.RCS_PEDESTRIAN_MIN <= rcs_value <= self.RCS_PEDESTRIAN_MAX:
            if relative_speed_magnitude <= self.SPEED_PEDESTRIAN_MAX:
                return ObjectType.PEDESTRIAN
            elif relative_speed_magnitude <= self.SPEED_CYCLIST_MAX:
                return ObjectType.CYCLIST
        
        # Default based on speed
        if relative_speed_magnitude <= self.SPEED_PEDESTRIAN_MAX:
            return ObjectType.PEDESTRIAN
        elif relative_speed_magnitude <= self.SPEED_CYCLIST_MAX:
            return ObjectType.CYCLIST
        else:
            return ObjectType.VEHICLE
            
        return ObjectType.UNKNOWN

    def calculate_overlap(self, obj_y_distance, obj_width_estimate=2.0):
        """Calculate overlap between vehicle path and object"""
        vehicle_path_half_width = self.VEHICLE_PATH_WIDTH / 2.0
        obj_half_width = obj_width_estimate / 2.0
        
        # Object boundaries
        obj_left = obj_y_distance - obj_half_width
        obj_right = obj_y_distance + obj_half_width
        
        # Vehicle path boundaries
        path_left = -vehicle_path_half_width
        path_right = vehicle_path_half_width
        
        # Calculate overlap
        overlap_left = max(path_left, obj_left)
        overlap_right = min(path_right, obj_right)
        
        if overlap_right > overlap_left:
            return overlap_right - overlap_left
        else:
            return 0.0

    def is_object_in_front(self, obj_y_distance):
        """Check if object is directly in front (within threshold of center line)"""
        return abs(obj_y_distance) <= self.IN_PATH_THRESHOLD

    def radar_callback(self, msg: RadarTrackList):
        """Process radar data for permanent stop logic"""
        self.closest_obstacle_distance = float('inf')
        self.closest_obstacle_type = ObjectType.UNKNOWN
        self.obstacle_in_path = False
        self.overlap_detected = False
        self.overlap_amount = 0.0
        
        for obj in msg.objects:
            # Only consider objects in front of vehicle and within monitoring range
            if (obj.x_distance > 0.1 and 
                obj.x_distance < self.RADAR_MONITORING_RANGE):
                
                # Classify the object
                obj_type = self.classify_object(obj)
                
                # Get appropriate width estimate
                if obj_type == ObjectType.PEDESTRIAN:
                    width_estimate = 0.6
                elif obj_type == ObjectType.CYCLIST:
                    width_estimate = 1.0
                elif obj_type == ObjectType.VEHICLE:
                    width_estimate = 2.0
                else:
                    width_estimate = 1.5
                
                # Calculate overlap with vehicle path
                overlap = self.calculate_overlap(obj.y_distance, width_estimate)
                
                # Check if object threatens path
                object_threatens_path = (
                    overlap >= self.ANY_OVERLAP_THRESHOLD or  # Any overlap
                    self.is_object_in_front(obj.y_distance)   # OR directly in front
                )
                
                if object_threatens_path:
                    self.overlap_detected = True
                    self.overlap_amount = max(self.overlap_amount, overlap)
                    
                    # Update closest obstacle
                    if obj.x_distance < self.closest_obstacle_distance:
                        self.closest_obstacle_distance = obj.x_distance
                        self.closest_obstacle_type = obj_type
                        self.obstacle_in_path = True

    def velocity_callback(self, msg: Velocity):
        """Update current vehicle speed"""
        self.current_speed = msg.vehicle_velocity

    def inertial_callback(self, msg: Pose):
        """Update distance traveled based on position"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        current_position = [msg.position.x, msg.position.y]
        
        if self.last_position is not None and self.last_time is not None:
            dx = current_position[0] - self.last_position[0]
            dy = current_position[1] - self.last_position[1]
            distance_step = math.sqrt(dx**2 + dy**2)
            
            dt = current_time - self.last_time
            if dt > 0 and distance_step > 0 and distance_step < (self.current_speed * dt * 2):
                self.distance_traveled += distance_step
        
        self.last_position = current_position
        self.last_time = current_time

    def determine_vehicle_state(self):
        """Determine current vehicle state with PERMANENT STOP logic"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Stage 1: If permanent stop is already triggered, NEVER auto-release
        if self.permanent_stop_triggered:
            self.get_logger().debug(f"PERMANENT STOP ACTIVE: {self.stop_trigger_reason}")
            return AEBState.PERMANENT_STOP
        
        # Stage 2: Check for NEW permanent stop triggers - DANGER ZONE 4-8m
        if (self.obstacle_in_path and 
            self.DANGER_ZONE_MIN <= self.closest_obstacle_distance <= self.DANGER_ZONE_MAX):
            
            # TRIGGER PERMANENT STOP
            if not self.permanent_stop_triggered:
                self.permanent_stop_triggered = True
                self.stop_trigger_reason = (f"{self.closest_obstacle_type.value} at {self.closest_obstacle_distance:.1f}m "
                                          f"with {self.overlap_amount:.1f}m overlap")
                self.stop_trigger_distance = self.closest_obstacle_distance
                self.permanent_stop_timestamp = current_time
                
                self.get_logger().warn(f"🚨 PERMANENT STOP TRIGGERED! {self.stop_trigger_reason}")
                self.get_logger().warn(f"🛑 VEHICLE WILL NOT MOVE AUTOMATICALLY - MANUAL OVERRIDE REQUIRED")
            
            return AEBState.PERMANENT_STOP
        
        # Stage 3: Emergency stop for very close obstacles (under 4m)
        if self.obstacle_in_path and self.closest_obstacle_distance < self.DANGER_ZONE_MIN:
            return AEBState.EMERGENCY_STOP
        
        # Stage 4: Normal braking for obstacles beyond danger zone (8-12m)
        if (self.obstacle_in_path and 
            self.closest_obstacle_distance > self.DANGER_ZONE_MAX and
            self.closest_obstacle_distance < (self.DANGER_ZONE_MAX * 1.5)):
            return AEBState.BRAKING
        
        # Stage 5: Normal operation states
        if (self.distance_traveled < self.ACCELERATION_DISTANCE and 
            self.current_speed < (self.TARGET_SPEED_MS - 0.5)):
            return AEBState.ACCELERATING
        
        return AEBState.CRUISING

    def calculate_control_outputs(self):
        """Calculate throttle and brake outputs with permanent stop priority"""
        self.throttle_output = 0.0
        self.brake_output = 0.0
        
        if self.current_state == AEBState.PERMANENT_STOP:
            # PERMANENT STOP - NO THROTTLE EVER, FULL BRAKE ALWAYS
            self.brake_output = 1.0
            self.throttle_output = 0.0
            
        elif self.current_state == AEBState.EMERGENCY_STOP:
            # Full braking for emergency stop
            self.brake_output = 1.0
            self.throttle_output = 0.0
            
        elif self.current_state == AEBState.BRAKING:
            # Progressive braking for obstacles at distance
            max_deceleration = 10.0
            brake_proportion = min(1.0, self.BRAKING_DECELERATION / max_deceleration)
            self.brake_output = brake_proportion
            self.throttle_output = 0.0
            
        elif self.current_state == AEBState.ACCELERATING:
            # Accelerate to reach target speed over 100 meters
            if self.ACCELERATION_DISTANCE > 0:
                remaining_distance = max(1.0, self.ACCELERATION_DISTANCE - self.distance_traveled)
                required_speed_squared = self.TARGET_SPEED_MS**2
                required_acceleration = required_speed_squared / (2 * remaining_distance)
                
                max_acceleration = 3.0  # m/s²
                required_acceleration = min(required_acceleration, max_acceleration)
                
                self.throttle_output = min(0.8, required_acceleration / max_acceleration)
            else:
                self.throttle_output = 0.3
                
        elif self.current_state == AEBState.CRUISING:
            # Maintain target speed
            speed_error = self.TARGET_SPEED_MS - self.current_speed
            
            if speed_error > 0.5:  # Need to accelerate
                self.throttle_output = min(0.4, speed_error * 0.2)
            elif speed_error < -0.5:  # Need to slow down
                self.brake_output = min(0.3, abs(speed_error) * 0.1)

    def control_loop(self):
        """Main control loop executed at 50Hz"""
        # Update vehicle state
        self.current_state = self.determine_vehicle_state()
        
        # Calculate control outputs
        self.calculate_control_outputs()
        
        # Create and publish control message
        control_msg = Control()
        control_msg.steering = self.steering_output
        control_msg.throttle = float(self.throttle_output)
        control_msg.brake = float(self.brake_output)
        control_msg.latswitch = 0  # Lateral control off
        control_msg.longswitch = 1  # Longitudinal control on
        
        self.control_pub.publish(control_msg)
        
        # Periodic logging
        if not hasattr(self, 'last_log_time'):
            self.last_log_time = 0
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_log_time > 1.0:  # Log every second
            self.last_log_time = current_time
            
            obstacle_status = "No obstacle"
            if self.obstacle_in_path:
                obstacle_status = f"Obstacle: {self.closest_obstacle_distance:.1f}m ({self.closest_obstacle_type.value})"
                if self.overlap_detected:
                    obstacle_status += f", Overlap: {self.overlap_amount:.1f}m"
            
            stop_status = ""
            if self.permanent_stop_triggered:
                hold_time = current_time - self.permanent_stop_timestamp if self.permanent_stop_timestamp else 0
                stop_status = f" | 🛑 PERMANENT_STOP: {self.stop_trigger_reason} (Active: {hold_time:.1f}s)"
            
            self.get_logger().info(
                f"State: {self.current_state.value} | "
                f"Speed: {self.current_speed*3.6:.1f} km/h (Target: {self.TARGET_SPEED_KMPH:.1f}) | "
                f"Distance: {self.distance_traveled:.1f}m | "
                f"{obstacle_status}{stop_status} | "
                f"Controls: T={self.throttle_output:.2f} B={self.brake_output:.2f}"
            )

def main(args=None):
    rclpy.init(args=args)
    aeb_controller = None
    
    try:
        aeb_controller = AEBController()
        rclpy.spin(aeb_controller)
        
    except KeyboardInterrupt:
        if aeb_controller:
            aeb_controller.get_logger().info("AEB Controller shutting down...")
            
    except Exception as e:
        if aeb_controller:
            aeb_controller.get_logger().error(f"Unhandled exception: {e}")
        else:
            print(f"Exception before node initialization: {e}")
            
    finally:
        if aeb_controller:
            try:
                final_control = Control()
                final_control.throttle = 0.0
                final_control.brake = 1.0
                final_control.steering = 0.0
                final_control.longswitch = 1
                final_control.latswitch = 0
                aeb_controller.control_pub.publish(final_control)
                aeb_controller.get_logger().info("Sent final stop command")
                time.sleep(0.1)
            except Exception as e:
                print(f"Error sending final command: {e}")
            
            aeb_controller.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()