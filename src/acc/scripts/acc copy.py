#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import time
from collections import deque
from enum import Enum

# Import required message types
from radar_msgs.msg import RadarTrackList
from feedback.msg import Velocity
from vehiclecontrol.msg import Control
from inertial_msgs.msg import Pose

class ACCState(Enum):
    DISABLED = 0
    STANDSTILL = 1
    CRUISING = 2
    FOLLOWING = 3
    EMERGENCY_BRAKING = 4

class ACCController(Node):
    def __init__(self):
        super().__init__('acc_controller')
        self.get_logger().info("Starting Final BAJA ACC Controller - Enhanced Braking Logic")
        self.get_logger().info("Version: 2025-06-13 19:14:29 | Developer: Ashish480")
        
        # BAJA Specific Parameters (Per Requirements)
        self.max_speed_kmph = 30.0
        self.max_speed_ms = self.max_speed_kmph / 3.6  # 8.33 m/s
        self.target_speed_kmph = 30.0 
        self.target_speed_ms = self.target_speed_kmph / 3.6
        
        # Distance Requirements - ENHANCED WITH 7.5M BRAKE TRIGGER
        self.desired_following_distance = 8.0  # TARGET: 8 meters at all times
        self.safety_brake_threshold = 7.5     # ENHANCED BRAKE: if distance < 7.5m
        self.distance_tolerance = 0.3         # Acceptable tolerance around 8m
        self.safety_margin = 2.0              # Never go below 2m (absolute minimum)
        
        # Performance Requirements
        self.acceleration_to_30kmh = 5.0  # Must reach 30 km/h in 5 seconds
        self.required_acceleration = self.max_speed_ms / self.acceleration_to_30kmh  # 1.67 m/s²
        
        # State transition thresholds - OPTIMIZED FOR EARLIER ENGAGEMENT
        self.follow_engage_threshold = 25.0  # Engage following earlier for better control
        self.cruise_resume_threshold = 35.0  # Resume cruise with hysteresis
        self.emergency_brake_distance = 2.0  # Emergency brake threshold
        self.min_gap = 2.0  # Absolute minimum gap
        
        # Control parameters - TUNED FOR STABILITY AND PERFORMANCE
        self.max_acceleration = 1.8  # Maximum acceleration
        self.max_deceleration = -3.0  # Maximum deceleration
        self.comfort_acceleration = 1.2  # Comfortable acceleration limit
        self.comfort_deceleration = -1.5  # Comfortable deceleration limit
        
        # ENHANCED BRAKING PARAMETERS FROM YOUR CODE
        self.enhanced_brake_deceleration = -2.5  # Strong braking when < 7.5m
        self.time_gap = 0.05  # Time gap for enhanced braking calculations
        self.min_gap_enhanced = 5.0  # Minimum gap for enhanced braking logic
        
        # Standstill and startup parameters
        self.standstill_speed_threshold = 0.3  # Speed threshold for standstill detection
        self.startup_acceleration_value = 0.8  # Gentle startup acceleration
        self.hold_brake_deceleration = -0.1  # Very gentle brake hold
        self.min_relative_speed_to_start_moving = 0.2  # Threshold for following startup
        
        # PID Controller parameters - FINAL OPTIMIZED VALUES
        
        # Speed control (Cruising) - Well-tuned for 30km/h target
        self.kp_speed = 1.0  # Good responsiveness
        self.ki_speed = 0.08  # Steady-state accuracy
        self.kd_speed = 0.1   # Damping for stability
        self.speed_integral_limit = 3.0
        
        # Gap control (Following) - ENHANCED FROM YOUR CODE
        self.kp_gap = 0.55    # Enhanced from your working braking logic
        self.ki_gap = 0.01    # Very low to prevent standstill issues
        self.kd_gap = 0.15    # Good derivative response
        self.kp_rel_vel = 0.65 # Strong relative velocity response (from your code)
        self.kp_lead_accel = 0.35  # Enhanced feedforward (from your code)
        self.gap_integral_limit = 5.0  # From your working code
        
        # Enhanced safety brake control gains for 7.5m threshold
        self.kp_safety_brake = 0.8  # Aggressive braking when < 7.5m
        self.kd_safety_brake = 0.6  # Strong damping for safety brake
        
        # Prediction parameters from your code
        self.prediction_horizon = 0.8  # From your working braking logic
        self.kp_pred_gap_error = 0.5   # Enhanced predictive control from your code
        
        # Advanced filtering for ultra-smooth operation
        self.acceleration_filter_alpha = 0.85  # Heavy filtering for stability
        self.command_rate_limit = 0.4  # Limit acceleration change rate
        
        # State variables
        self.current_speed_ms = 0.0
        self.current_speed_kmph = 0.0
        self.current_acceleration_commanded = 0.0 
        self.leading_vehicle = None
        self.leading_distance = float('inf')
        self.leading_relative_speed = 0.0
        self.leading_vehicle_accel_x = 0.0
        self.leading_vehicle_speed_abs = 0.0
        
        # State management with hysteresis
        self.acc_enabled = False
        self.acc_state = ACCState.DISABLED
        self.previous_acc_state = ACCState.DISABLED
        self.state_entry_time = 0.0
        self.min_state_duration = 0.5  # Minimum time in state before transitions
        
        # PID state variables
        self.speed_error_integral = 0.0
        self.speed_error_previous = 0.0
        self.gap_error_integral = 0.0
        self.gap_error_previous = 0.0
        
        # Filtering and smoothing
        self.acceleration_filter = 0.0
        self.previous_acceleration_command = 0.0
        
        # Enhanced safety brake tracking
        self.enhanced_brake_active = False
        self.last_enhanced_brake_log = 0.0
        
        # Timing
        self.last_time = self.get_clock().now().nanoseconds / 1e9
        
        # Speed profile for scenarios
        self.speed_profile_active = False
        self.target_speed_profile = []
        self.profile_start_time = None
        
        # ROS2 subscriptions and publishers
        self.radar_sub = self.create_subscription(
            RadarTrackList, 'RadarObjects', self.radar_callback, 10)
        self.velocity_sub = self.create_subscription(
            Velocity, 'VehicleSpeed', self.velocity_callback, 10)
        
        self.control_pub = self.create_publisher(Control, '/vehicle_control', 10)
        self.timer = self.create_timer(0.02, self.control_loop)  # 50Hz control loop
        
        self.get_logger().info("Final BAJA ACC Controller initialized:")
        self.get_logger().info(f"- Max Speed: {self.max_speed_kmph} km/h")
        self.get_logger().info(f"- Target Following Distance: {self.desired_following_distance}m")
        self.get_logger().info(f"- Enhanced Brake Threshold: {self.safety_brake_threshold}m")
        self.get_logger().info(f"- Enhanced braking logic from working codebase integrated")
        self.get_logger().info(f"- Required 0-30km/h time: {self.acceleration_to_30kmh}s")

    def _reset_speed_pid_integrals(self):
        """Reset speed control PID states"""
        self.speed_error_integral = 0.0
        self.speed_error_previous = 0.0
        self.get_logger().debug("Speed PID integrals reset")

    def _reset_gap_pid_integrals(self):
        """Reset gap control PID states"""
        self.gap_error_integral = 0.0
        self.gap_error_previous = 0.0
        self.get_logger().debug("Gap PID integrals reset")

    def _reset_pid_integrals(self):
        """Reset all PID controller states"""
        self._reset_speed_pid_integrals()
        self._reset_gap_pid_integrals()
        self.acceleration_filter = 0.0
        self.previous_acceleration_command = 0.0
        self.enhanced_brake_active = False
        self.get_logger().debug("All PID states reset")

    def radar_callback(self, msg: RadarTrackList):
        """Process radar data to find closest valid vehicle"""
        closest_distance = float('inf')
        closest_vehicle_obj = None
        
        for obj in msg.objects:
            # Enhanced filtering from your working code
            if obj.x_distance > 0.1 and abs(obj.y_distance) < 2.0:
                if obj.x_distance < closest_distance:
                    closest_distance = obj.x_distance
                    closest_vehicle_obj = obj

        # Update leading vehicle information
        if closest_vehicle_obj:
            old_distance = self.leading_distance
            self.leading_vehicle = closest_vehicle_obj 
            self.leading_distance = closest_vehicle_obj.x_distance
            self.leading_relative_speed = closest_vehicle_obj.vx 
            self.leading_vehicle_speed_abs = self.current_speed_ms + self.leading_relative_speed

            try:
                self.leading_vehicle_accel_x = closest_vehicle_obj.ax 
            except AttributeError:
                self.leading_vehicle_accel_x = 0.0 
            
            # Log only significant changes to reduce noise
            if abs(old_distance - self.leading_distance) > 3.0:
                self.get_logger().debug(
                    f"Lead vehicle: {self.leading_distance:.1f}m, "
                    f"RelVel: {self.leading_relative_speed:.1f}m/s, "
                    f"LeadSpeed: {self.leading_vehicle_speed_abs*3.6:.1f}km/h"
                )
        else:
            if self.leading_vehicle is not None:
                self.get_logger().info("Lead vehicle lost")
            self._clear_leading_vehicle()

    def _clear_leading_vehicle(self):
        """Clear all leading vehicle data"""
        self.leading_vehicle = None
        self.leading_distance = float('inf')
        self.leading_relative_speed = 0.0
        self.leading_vehicle_accel_x = 0.0
        self.leading_vehicle_speed_abs = 0.0
        self.enhanced_brake_active = False
            
    def velocity_callback(self, msg: Velocity):
        """Update ego vehicle speed"""
        self.current_speed_ms = max(0.0, msg.vehicle_velocity)  # Ensure non-negative
        self.current_speed_kmph = self.current_speed_ms * 3.6
        
        # Update leading vehicle absolute speed if present
        if self.leading_vehicle:
            self.leading_vehicle_speed_abs = self.current_speed_ms + self.leading_relative_speed
    
    def set_speed_profile(self, speed_profile):
        """Set speed profile for specific scenarios"""
        self.speed_profile_active = True
        self.target_speed_profile = speed_profile
        self.profile_start_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(f"Speed profile activated: {speed_profile}")
        
    def get_target_speed_from_profile(self):
        """Get target speed from profile or default"""
        if not self.speed_profile_active or not self.target_speed_profile:
            return self.target_speed_ms
        
        elapsed_time = (self.get_clock().now().nanoseconds / 1e9) - self.profile_start_time
        
        for time_point, speed_kmph in self.target_speed_profile:
            if elapsed_time <= time_point:
                return speed_kmph / 3.6
        
        return self.target_speed_profile[-1][1] / 3.6
    
    def _get_desired_following_distance(self):
        """Return constant 8-meter following distance regardless of speed"""
        return self.desired_following_distance  # Always 8 meters

    def _calculate_safe_following_distance(self, ego_speed_ms):
        """Enhanced safe distance calculation from your working code"""
        calculated_gap = self.desired_following_distance + ego_speed_ms * self.time_gap
        safe_dist = max(self.min_gap_enhanced, calculated_gap)
        return safe_dist

    def _predict_future_states(self, current_commanded_accel):
        """Enhanced prediction logic from your working braking code"""
        t_h = self.prediction_horizon 

        ego_speed_future = self.current_speed_ms + current_commanded_accel * t_h
        ego_speed_future = max(0, ego_speed_future) 
        ego_dist_traveled_future = self.current_speed_ms * t_h + 0.5 * current_commanded_accel * (t_h**2)

        if self.leading_vehicle:
            v_lead_current_abs = self.leading_relative_speed + self.current_speed_ms 
            lead_speed_future = v_lead_current_abs + self.leading_vehicle_accel_x * t_h
            lead_speed_future = max(0, lead_speed_future)
            lead_dist_traveled_future = v_lead_current_abs * t_h + 0.5 * self.leading_vehicle_accel_x * (t_h**2)
            predicted_gap = (self.leading_distance + lead_dist_traveled_future) - ego_dist_traveled_future
        else: 
            predicted_gap = float('inf')

        predicted_safe_gap = self._calculate_safe_following_distance(ego_speed_future)
        predicted_gap_error = predicted_gap - predicted_safe_gap 

        self.get_logger().debug(
            f"PREDICT: Horizon={t_h:.2f}s. EgoSpdFut={ego_speed_future:.2f}. "
            f"PredGap={predicted_gap:.2f}, PredSafeGap={predicted_safe_gap:.2f}, PredGapErr={predicted_gap_error:.2f}"
        )
        return predicted_gap_error

    def _check_enhanced_brake_condition(self):
        """Check if enhanced brake should be applied when distance < 7.5m"""
        if not self.leading_vehicle:
            return False
        
        # Apply enhanced brake if distance is below 7.5m threshold
        if self.leading_distance < self.safety_brake_threshold:
            # Log enhanced brake activation (throttled to once per second)
            current_time = self.get_clock().now().nanoseconds / 1e9
            if current_time - self.last_enhanced_brake_log > 1.0:
                self.get_logger().warn(f"ENHANCED BRAKE ACTIVE: Distance {self.leading_distance:.1f}m < {self.safety_brake_threshold}m threshold")
                self.last_enhanced_brake_log = current_time
            return True
        
        return False

    def _update_acc_state(self):
        """State machine with improved emergency brake exit logic"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        time_in_state = current_time - self.state_entry_time
        self.previous_acc_state = self.acc_state
        new_state = self.acc_state
        
        # Prevent rapid state changes with minimum duration requirement
        if time_in_state < self.min_state_duration and self.acc_state != ACCState.DISABLED:
            return  # Stay in current state
        
        # ENHANCED STATE MACHINE LOGIC WITH 7.5M ENHANCED BRAKE
        if not self.acc_enabled:
            new_state = ACCState.DISABLED
        elif self.current_speed_ms < self.standstill_speed_threshold:
            # HIGHEST PRIORITY: If ego vehicle is stopped, go to standstill
            # This allows recovery from emergency braking when both vehicles are stopped
            new_state = ACCState.STANDSTILL
            if self.acc_state == ACCState.EMERGENCY_BRAKING:
                self.get_logger().info("Emergency brake complete - transitioning to standstill for distance recovery")
        elif (self.leading_vehicle and 
              self.leading_distance < self.emergency_brake_distance and
              self.current_speed_ms > 1.0 and  # Only emergency brake if we're moving
              abs(self.leading_relative_speed) > -2.0):  # And not if lead is moving away fast
            new_state = ACCState.EMERGENCY_BRAKING
        else:
            # Moving states with hysteresis
            if self.leading_vehicle:
                if (self.acc_state != ACCState.FOLLOWING and 
                    self.leading_distance < self.follow_engage_threshold):
                    new_state = ACCState.FOLLOWING
                elif (self.acc_state == ACCState.FOLLOWING and 
                      self.leading_distance > self.cruise_resume_threshold):
                    new_state = ACCState.CRUISING
                elif self.acc_state == ACCState.FOLLOWING:
                    new_state = ACCState.FOLLOWING  # Stay in following
                else:
                    new_state = ACCState.CRUISING
            else:
                new_state = ACCState.CRUISING
        
        # Handle state transitions
        if new_state != self.acc_state:
            self.get_logger().info(f"ACC State: {self.acc_state.name} -> {new_state.name}")
            self.state_entry_time = current_time
            
            # Reset appropriate controllers on state change
            if new_state == ACCState.FOLLOWING:
                self._reset_gap_pid_integrals()
            elif new_state == ACCState.CRUISING:
                self._reset_speed_pid_integrals()
            elif new_state in [ACCState.STANDSTILL, ACCState.EMERGENCY_BRAKING]:
                self._reset_pid_integrals()
                
            self.acc_state = new_state

    def _calculate_standstill_acceleration(self, dt):
        """Handle standstill: actively achieve and maintain 8m distance"""
        target_speed = self.get_target_speed_from_profile()
        
        if self.leading_vehicle:
            distance_error = self.leading_distance - self.desired_following_distance
            
            # If we're very close (emergency brake aftermath), be careful
            if self.leading_distance < 3.0:
                if abs(self.leading_relative_speed) < 0.1:  # Both vehicles stationary
                    # Hold position and wait - don't try to move yet
                    self.get_logger().debug(f"STANDSTILL: Very close ({self.leading_distance:.1f}m), waiting for lead vehicle")
                    return -0.02  # Very gentle brake hold
                else:
                    return self.hold_brake_deceleration
            
            # If we're significantly short of 8m (3-6m range)
            elif (distance_error < -2.0 and  # More than 2m short of 8m
                  abs(self.leading_relative_speed) < 0.2 and  # Lead is nearly stationary
                  self.leading_distance > 3.0):  # Safe distance to start moving
                # Gentle forward creep to reach 8m
                creep_strength = min(0.3, abs(distance_error) * 0.1)  # Proportional to error
                self.get_logger().debug(f"STANDSTILL: Moving forward to reach 8m (current: {self.leading_distance:.1f}m, creep: {creep_strength:.2f})")
                return creep_strength
            
            # If we're moderately short (6-7.5m range)
            elif (distance_error < -0.5 and 
                  abs(self.leading_relative_speed) < 0.2):
                return 0.1  # Very gentle forward nudge
            
            # If lead vehicle starts moving, follow
            elif self.leading_relative_speed > 0.3:
                self.get_logger().debug("STANDSTILL: Lead vehicle moving, starting to follow")
                return self.startup_acceleration_value
            
            # If we're at good distance (7.5-8.5m), hold gently
            elif abs(distance_error) < 0.5:
                return self.hold_brake_deceleration * 0.3
            
            # Otherwise, gentle hold
            else:
                return self.hold_brake_deceleration * 0.5
        else:
            # No lead vehicle - start cruising if target speed > 0
            if target_speed > 0.5:
                self.get_logger().debug("STANDSTILL: No lead vehicle, starting to cruise")
                return self.startup_acceleration_value
            else:
                return self.hold_brake_deceleration

    def _calculate_cruising_acceleration(self, target_speed, dt):
        """Speed control for cruising mode"""
        # Limit target speed to maximum allowed
        target_speed = min(target_speed, self.max_speed_ms)
        
        # PID speed control
        speed_error = target_speed - self.current_speed_ms
        
        self.speed_error_integral += speed_error * dt
        self.speed_error_integral = max(min(self.speed_error_integral, self.speed_integral_limit),
                                       -self.speed_integral_limit)
        
        speed_error_derivative = 0.0
        if dt > 0.001:
            speed_error_derivative = (speed_error - self.speed_error_previous) / dt
        self.speed_error_previous = speed_error
        
        desired_acceleration = (self.kp_speed * speed_error + 
                               self.ki_speed * self.speed_error_integral + 
                               self.kd_speed * speed_error_derivative)
        
        # Apply comfort limits
        if desired_acceleration < 0:
            desired_acceleration = max(desired_acceleration, self.comfort_deceleration)
        elif desired_acceleration > 0:
            desired_acceleration = min(desired_acceleration, self.comfort_acceleration)
        
        return desired_acceleration

    def _calculate_following_acceleration(self, target_speed, dt):
        """Enhanced following control with 7.5m enhanced braking logic from your code"""
        if not self.leading_vehicle:
            return self._calculate_cruising_acceleration(target_speed, dt)

        # Check for enhanced brake condition (distance < 7.5m)
        enhanced_brake_needed = self._check_enhanced_brake_condition()
        
        if enhanced_brake_needed:
            # ENHANCED BRAKING LOGIC FROM YOUR WORKING CODE
            self.enhanced_brake_active = True
            
            # Use your working enhanced safe distance calculation
            current_safe_dist = self._calculate_safe_following_distance(self.current_speed_ms)
            gap_error = self.leading_distance - current_safe_dist 
            
            # Your working PID control for enhanced braking
            self.gap_error_integral += gap_error * dt
            self.gap_error_integral = max(min(self.gap_error_integral, self.gap_integral_limit), -self.gap_integral_limit)

            gap_error_derivative = 0.0
            if dt > 0.001: 
                gap_error_derivative = (gap_error - self.gap_error_previous) / dt
            self.gap_error_previous = gap_error

            pid_gap_output = (self.kp_gap * gap_error + 
                              self.ki_gap * self.gap_error_integral + 
                              self.kd_gap * gap_error_derivative)
            
            # Your working relative speed and feedforward components
            relative_speed_component = self.kp_rel_vel * self.leading_relative_speed 
            lead_accel_component = self.kp_lead_accel * self.leading_vehicle_accel_x
            
            base_desired_acceleration = pid_gap_output + relative_speed_component + lead_accel_component
            
            # Your working predictive control
            predicted_gap_error = self._predict_future_states(base_desired_acceleration) 
            predictive_accel_adjustment = self.kp_pred_gap_error * predicted_gap_error
            
            enhanced_brake_acceleration = base_desired_acceleration + predictive_accel_adjustment
            
            # Apply your working speed limit logic
            if self.current_speed_ms > target_speed and enhanced_brake_acceleration > 0:
                speed_limit_factor = -0.5 * (self.current_speed_ms - target_speed) 
                if enhanced_brake_acceleration > speed_limit_factor: 
                    enhanced_brake_acceleration = speed_limit_factor

            # Apply enhanced brake limits
            enhanced_brake_acceleration = max(self.enhanced_brake_deceleration, 
                                            min(self.comfort_acceleration, enhanced_brake_acceleration))
            
            self.get_logger().debug(f"ENHANCED BRAKE: Distance={self.leading_distance:.1f}m < {self.safety_brake_threshold:.1f}m, SafeDist={current_safe_dist:.1f}m, Accel={enhanced_brake_acceleration:.2f}m/s²")
            
            return enhanced_brake_acceleration
        
        else:
            # NORMAL FOLLOWING CONTROL (distance >= 7.5m) - Your optimized logic
            self.enhanced_brake_active = False
            
            # Conservative PID control for distance
            distance_error = self.leading_distance - self.desired_following_distance
            
            self.gap_error_integral += distance_error * dt
            self.gap_error_integral = max(min(self.gap_error_integral, 1.0), -1.0)  # Tight limit
            
            gap_error_derivative = 0.0
            if dt > 0.001:
                gap_error_derivative = (distance_error - self.gap_error_previous) / dt
            self.gap_error_previous = distance_error
            
            # Conservative PID distance control
            distance_control = (0.25 * distance_error +           # Conservative P-gain
                               0.025 * self.gap_error_integral +  # Small I-gain
                               0.4 * gap_error_derivative)        # Strong D-gain for damping
            
            # Gentle relative velocity control
            relative_velocity_control = 0.35 * self.leading_relative_speed
            
            # Minimal feedforward
            feedforward_control = 0.1 * self.leading_vehicle_accel_x
            
            # Combine control components
            total_acceleration = distance_control + relative_velocity_control + feedforward_control
            
            # Apply speed limits
            target_speed = min(target_speed, self.max_speed_ms)
            if self.current_speed_ms > target_speed and total_acceleration > 0:
                speed_excess = self.current_speed_ms - target_speed
                total_acceleration = min(total_acceleration, -0.2 * speed_excess)
            
            # Apply comfort limits
            total_acceleration = max(self.comfort_deceleration, 
                                   min(self.comfort_acceleration, total_acceleration))
            
            # Additional smoothing for close distances
            if self.leading_distance < 10.0:
                total_acceleration *= 0.85
            
            return total_acceleration

    def calculate_desired_acceleration(self):
        """Main control calculation with enhanced smoothing and braking logic"""
        current_time_sec = self.get_clock().now().nanoseconds / 1e9
        dt = current_time_sec - self.last_time
        if dt <= 0.001:
            dt = 0.02  # Default to 50Hz if timing is off
        self.last_time = current_time_sec
        
        # Update state machine
        self._update_acc_state()
        
        # Get target speed
        target_speed = self.get_target_speed_from_profile()
        target_speed = min(target_speed, self.max_speed_ms)
        
        # Calculate base acceleration based on current state
        if self.acc_state == ACCState.DISABLED:
            raw_acceleration = 0.0
        elif self.acc_state == ACCState.EMERGENCY_BRAKING:
            raw_acceleration = self.max_deceleration
            self.get_logger().warn(f"EMERGENCY BRAKING: Distance={self.leading_distance:.1f}m")
        elif self.acc_state == ACCState.STANDSTILL:
            raw_acceleration = self._calculate_standstill_acceleration(dt)
        elif self.acc_state == ACCState.CRUISING:
            raw_acceleration = self._calculate_cruising_acceleration(target_speed, dt)
        elif self.acc_state == ACCState.FOLLOWING:
            raw_acceleration = self._calculate_following_acceleration(target_speed, dt)
        else:
            raw_acceleration = 0.0
        
        # Apply absolute limits
        limited_acceleration = max(self.max_deceleration, 
                                  min(self.max_acceleration, raw_acceleration))
        
        # Rate limiting (but allow enhanced brake to be more responsive)
        if not self.enhanced_brake_active:
            acceleration_change = limited_acceleration - self.previous_acceleration_command
            max_change = self.command_rate_limit * dt * 50
            
            if abs(acceleration_change) > max_change:
                if acceleration_change > 0:
                    limited_acceleration = self.previous_acceleration_command + max_change
                else:
                    limited_acceleration = self.previous_acceleration_command - max_change
        
        # Filtering (less filtering during enhanced brake for responsiveness)
        if self.enhanced_brake_active:
            self.acceleration_filter = (0.7 * self.acceleration_filter + 
                                       0.3 * limited_acceleration)
        else:
            self.acceleration_filter = (self.acceleration_filter_alpha * self.acceleration_filter + 
                                       (1 - self.acceleration_filter_alpha) * limited_acceleration)
        
        self.previous_acceleration_command = self.acceleration_filter
        self.current_acceleration_commanded = self.acceleration_filter
        
        return self.acceleration_filter
    
    def acceleration_to_control_signals(self, desired_acceleration):
        """Convert acceleration to throttle/brake with proper dead zone"""
        throttle, brake = 0.0, 0.0
        
        # Smaller dead zone during enhanced brake for better responsiveness
        dead_zone = 0.05 if self.enhanced_brake_active else 0.15
        
        if abs(desired_acceleration) < dead_zone:
            return throttle, brake
        
        if desired_acceleration > dead_zone:
            # Throttle mapping
            throttle = min(1.0, desired_acceleration / self.max_acceleration)
        elif desired_acceleration < -dead_zone:
            # Enhanced brake mapping for better responsiveness
            brake = min(1.0, abs(desired_acceleration) / abs(self.max_deceleration))
        
        return throttle, brake
    
    def control_loop(self):
        """Main control loop - 50Hz execution"""
        if not self.acc_enabled:
            # Publish zero controls when disabled
            control_msg = Control(throttle=0.0, brake=0.0, steering=0.0, 
                                longswitch=0, latswitch=0)
            self.control_pub.publish(control_msg)
            return
        
        # Calculate desired acceleration
        desired_acceleration = self.calculate_desired_acceleration()
        
        # Convert to control signals
        throttle, brake = self.acceleration_to_control_signals(desired_acceleration)
        
        # Create and publish control message
        control_msg = Control()
        control_msg.throttle = throttle
        control_msg.brake = brake
        control_msg.steering = 0.0  # Lateral control handled by IPG Driver
        control_msg.longswitch = 1 if self.acc_enabled else 0
        control_msg.latswitch = 0
        
        self.control_pub.publish(control_msg)
        
        # Periodic logging for monitoring
        if not hasattr(self, '_log_counter'):
            self._log_counter = 0
        self._log_counter += 1
        
        if self._log_counter % 50 == 0:  # Every 1 second at 50Hz
            target_speed = self.get_target_speed_from_profile()
            
            if self.leading_vehicle:
                desired_gap = self._get_desired_following_distance()
                gap_error = self.leading_distance - desired_gap
                brake_status = "🔴 ENHANCED BRAKE" if self.enhanced_brake_active else "🟢 NORMAL"
                lead_info = (f"Lead: {self.leading_distance:.1f}m "
                           f"(target: {desired_gap:.1f}m, error: {gap_error:+.1f}m) "
                           f"@ {self.leading_vehicle_speed_abs*3.6:.1f}km/h | {brake_status}")
            else:
                lead_info = "No Lead"
            
            self.get_logger().info(
                f"[{self.acc_state.name}] Speed: {self.current_speed_kmph:.1f}/{target_speed*3.6:.1f}km/h | "
                f"Accel: {desired_acceleration:.2f}m/s² | T:{throttle:.2f} B:{brake:.2f} | {lead_info}"
            )
    
    def enable_acc(self):
        """Enable ACC system"""
        if not self.acc_enabled:
            self.acc_enabled = True
            self._reset_pid_integrals()
            self.get_logger().info("ACC ENABLED - Enhanced braking logic for 7.5m threshold integrated")
        
    def disable_acc(self):
        """Disable ACC system"""
        if self.acc_enabled:
            self.acc_enabled = False
            self.acc_state = ACCState.DISABLED
            self._reset_pid_integrals()
            try:
                control_msg = Control(throttle=0.0, brake=0.0, steering=0.0, 
                                    longswitch=0, latswitch=0)
                self.control_pub.publish(control_msg)
            except:
                pass
            self.get_logger().info("ACC DISABLED")
    
    def set_target_speed(self, speed_kmph):
        """Set target cruising speed"""
        try:
            speed_kmph = float(speed_kmph)
            speed_kmph = max(0.0, min(speed_kmph, self.max_speed_kmph))
            self.target_speed_kmph = speed_kmph
            self.target_speed_ms = speed_kmph / 3.6
            self.get_logger().info(f"Target speed: {speed_kmph:.1f} km/h (8m target, enhanced brake @ 7.5m)")
        except ValueError:
            self.get_logger().error(f"Invalid speed: {speed_kmph}")
    
    def configure_for_scenario(self, scenario_type):
        """Configure ACC for specific BAJA scenarios"""
        self.get_logger().info(f"Configuring for BAJA scenario: {scenario_type}")
        self.speed_profile_active = False
        
        if scenario_type == "stop_and_go_1":
            self.set_target_speed(30.0)
        elif scenario_type == "stop_and_go_2":
            profile = [(3.0, 20.0), (5.0, 30.0), (12.0, 30.0), (14.0, 15.0), (20.0, 15.0)]
            self.set_speed_profile(profile)
        elif scenario_type == "stop_and_go_3":
            profile = [(5.0, 20.0), (7.0, 30.0), (12.0, 30.0), (15.0, 10.0), (17.0, 10.0), (22.0, 0.0)]
            self.set_speed_profile(profile)
        elif scenario_type == "cut_in":
            self.set_target_speed(30.0)
        elif scenario_type == "cut_out":
            self.set_target_speed(30.0)
        else:
            self.set_target_speed(30.0)

def main(args=None):
    rclpy.init(args=args)
    acc_controller = None
    
    try:
        acc_controller = ACCController()
        acc_controller.get_logger().info("=" * 80)
        acc_controller.get_logger().info("🚗 FINAL BAJA ACC - ENHANCED BRAKING EDITION 🚗")
        acc_controller.get_logger().info("=" * 80)
        acc_controller.get_logger().info("✅ Features:")
        acc_controller.get_logger().info("   • 8m constant target distance control")
        acc_controller.get_logger().info("   • Enhanced braking logic @ 7.5m threshold")
        acc_controller.get_logger().info("   • Working braking algorithm integrated")
        acc_controller.get_logger().info("   • Predictive control for stopping")
        acc_controller.get_logger().info("   • Emergency brake recovery system")
        acc_controller.get_logger().info("=" * 80)
        acc_controller.get_logger().info("👨‍💻 Developer: Ashish480")
        acc_controller.get_logger().info("📅 Date: 2025-06-13 19:14:29 UTC")
        acc_controller.get_logger().info("🏁 Ready for BAJA Virtual Dynamic Event")
        acc_controller.get_logger().info("=" * 80)
        acc_controller.get_logger().info("⏳ Starting in 3 seconds...")
        
        time.sleep(3.0)
        
        acc_controller.set_target_speed(30.0)
        acc_controller.enable_acc()
        
        acc_controller.get_logger().info("🚀 ENHANCED BAJA ACC Controller ACTIVE! 🚀")
        
        rclpy.spin(acc_controller)
        
    except KeyboardInterrupt:
        if acc_controller:
            acc_controller.get_logger().info("👋 Shutting down Enhanced BAJA ACC...")
    except Exception as e:
        if acc_controller:
            acc_controller.get_logger().error(f"💥 Error: {e}")
    finally:
        if acc_controller:
            acc_controller.disable_acc()
            try:
                acc_controller.destroy_node()
            except:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass
        print("✅ Enhanced BAJA ACC Controller shutdown complete!")
        print("🎯 Enhanced braking logic successfully integrated!")
        print("👨‍💻 Ashish480 - Thank you for using the enhanced system!")

if __name__ == '__main__':
    main()