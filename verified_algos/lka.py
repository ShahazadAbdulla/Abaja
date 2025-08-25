#!/usr/bin/env python3
# Lane Keeping Assist with Adaptive Stanley Controller
# Current Date and Time (UTC): 2025-06-20 00:44:03
# Current User's Login: Ashish480

import rclpy
from rclpy.node import Node
import numpy as np
from time import time as get_current_timestamp 
import math
import json 
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor
import datetime

# Import required message types
from std_msgs.msg import String, Float64, Bool
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from vehiclecontrol.msg import Control
from inertial_msgs.msg import Pose
from feedback.msg import Velocity

class PIDController:
    def __init__(self, kp, ki, kd, integral_limit=1.0, output_limit=1.0, derivative_filter_alpha=0.7):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0
        self.integral_limit = integral_limit
        self.output_limit = output_limit 
        self.derivative_filter_alpha = derivative_filter_alpha
        self.last_derivative = 0.0

    def compute(self, error, dt):
        if dt <= 0.0001: return 0.0
        p_term = self.kp * error
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)
        i_term = self.ki * self.integral
        derivative = (error - self.prev_error) / dt
        filtered_derivative = (self.derivative_filter_alpha * self.last_derivative) + \
                              (1 - self.derivative_filter_alpha) * derivative
        d_term = self.kd * filtered_derivative
        self.prev_error = error
        self.last_derivative = filtered_derivative
        output = p_term + i_term + d_term
        return np.clip(output, -self.output_limit, self.output_limit)

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_derivative = 0.0

class LaneKeepingAssistNode(Node):
    def __init__(self):
        super().__init__('lane_keeping_assist_node')

        self.user = "Ashish480"
        self.start_date_str = "2025-06-20 00:44:03"
        
        # --- ADAPTIVE PARAMETERS FOR LANE-RESPONSIVE CONTROL ---
        self.declare_parameter('stanley_k', 0.3, ParameterDescriptor(description='Stanley gain - RESPONSIVE to lane changes'))
        self.declare_parameter('stanley_softening', 1.0, ParameterDescriptor(description='Stanley softening factor'))
        
        # PID parameters for fine-tuning
        self.declare_parameter('pid_cte_kp', 0.015, ParameterDescriptor(description='PID P-gain - moderate'))
        self.declare_parameter('pid_cte_ki', 0.001, ParameterDescriptor(description='PID I-gain - small'))
        self.declare_parameter('pid_cte_kd', 0.005, ParameterDescriptor(description='PID D-gain - small'))
        self.declare_parameter('pid_cte_integral_limit', 0.15, ParameterDescriptor(description='PID I-term limit'))
        self.declare_parameter('pid_cte_output_limit', np.radians(3.0), ParameterDescriptor(description='Max PID contribution'))
        
        # Adaptive lookahead
        self.declare_parameter('lookahead_base_m', 6.0, ParameterDescriptor(description='Base lookahead distance'))
        self.declare_parameter('lookahead_speed_factor', 0.4, ParameterDescriptor(description='Speed-adaptive factor'))
        self.declare_parameter('min_lookahead_m', 3.0, ParameterDescriptor(description='Minimum lookahead'))
        self.declare_parameter('max_lookahead_m', 15.0, ParameterDescriptor(description='Maximum lookahead'))

        self.declare_parameter('path_smoothing_window', 5, ParameterDescriptor(description='Path smoothing window'))
        
        # RESPONSIVE STEERING LIMITS
        self.declare_parameter('max_lka_steering_wheel_angle_rad', np.radians(12.0), ParameterDescriptor(description='Max steering - 12 degrees'))
        self.declare_parameter('steering_rate_limit_rad_s', np.radians(25.0), ParameterDescriptor(description='Max steering rate - 25 deg/s'))
        
        # Responsiveness parameters
        self.declare_parameter('steering_smoothing_alpha', 0.7, ParameterDescriptor(description='Steering smoothing - MORE RESPONSIVE'))
        self.declare_parameter('min_control_speed_mps', 0.5, ParameterDescriptor(description='Minimum speed for control'))
        self.declare_parameter('cte_deadzone_m', 0.1, ParameterDescriptor(description='CTE deadzone to prevent oscillation'))

        self.declare_parameter('lka_master_enabled', True, ParameterDescriptor(description='Master LKA enable'))
        
        self._load_and_log_parameters()
        
        self.pid_cte_controller = PIDController(
            kp=self.pid_cte_kp, ki=self.pid_cte_ki, kd=self.pid_cte_kd,
            integral_limit=self.pid_cte_integral_limit,
            output_limit=self.pid_cte_output_limit
        )
        
        self.add_on_set_parameters_callback(self.parameters_callback_reconfigure_pid)
        
        # Vehicle state
        self.vehicle_x_map = 0.0
        self.vehicle_y_map = 0.0
        self.vehicle_yaw_map = 0.0 
        self.vehicle_speed_mps = 0.0
        self.last_steering_cmd_rad = 0.0
        self.smoothed_steering_cmd_rad = 0.0
        
        # Path tracking variables
        self.raw_path_waypoints = []
        self.smoothed_path_waypoints = []
        self.is_reference_path_available = False
        self.current_cte = 0.0
        self.current_heading_error = 0.0
        
        self.last_control_loop_time = get_current_timestamp()
        self._lka_log_counter = 0
        
        # Debug variables
        self.last_vehicle_x = 0.0
        self.last_vehicle_y = 0.0
        self.path_update_count = 0
        
        # Subscribers
        self.pose_subscriber = self.create_subscription(Pose, '/InertialData', self.pose_callback, 10)
        self.speed_subscriber = self.create_subscription(Velocity, '/VehicleSpeed', self.vehicle_speed_callback, 10)
        self.lane_subscriber = self.create_subscription(String, '/lane_coordinates', self.lane_coordinates_callback, 10)
        self.lka_enable_subscriber = self.create_subscription(Bool, '/lka/enable', self.lka_enable_callback, 10)

        # Publishers
        self.control_publisher = self.create_publisher(Control, '/vehicle_control', 10)
        self.processed_path_publisher = self.create_publisher(Path, '/lka/processed_path', 10)
        
        self.timer_period = 0.05  # 20 Hz control loop
        self.control_timer = self.create_timer(self.timer_period, self.control_loop)
        
        print("🚗" + "="*80)
        print("🛣️  ADAPTIVE STANLEY LANE KEEPING ASSIST")
        print(f"👤 User: {self.user}")
        print(f"🕐 Started: {self.start_date_str}")
        print(f"🎯 RESPONSIVE MODE: Stanley K = {self.stanley_k:.2f}, Max Steer = {math.degrees(self.max_lka_steering_wheel_angle_rad):.0f}°")
        print(f"📍 LANE-ADAPTIVE: Updates steering based on real-time lane position")
        print("🚗" + "="*80)

    def _load_and_log_parameters(self):
        self.stanley_k = float(self.get_parameter('stanley_k').value)
        self.stanley_softening = float(self.get_parameter('stanley_softening').value)
        self.pid_cte_kp = float(self.get_parameter('pid_cte_kp').value)
        self.pid_cte_ki = float(self.get_parameter('pid_cte_ki').value)
        self.pid_cte_kd = float(self.get_parameter('pid_cte_kd').value)
        self.pid_cte_integral_limit = float(self.get_parameter('pid_cte_integral_limit').value)
        self.pid_cte_output_limit = float(self.get_parameter('pid_cte_output_limit').value)
        self.lookahead_base_m = float(self.get_parameter('lookahead_base_m').value)
        self.lookahead_speed_factor = float(self.get_parameter('lookahead_speed_factor').value)
        self.min_lookahead_m = float(self.get_parameter('min_lookahead_m').value)
        self.max_lookahead_m = float(self.get_parameter('max_lookahead_m').value)
        self.path_smoothing_window = int(self.get_parameter('path_smoothing_window').value)
        self.max_lka_steering_wheel_angle_rad = float(self.get_parameter('max_lka_steering_wheel_angle_rad').value)
        self.steering_rate_limit_rad_s = float(self.get_parameter('steering_rate_limit_rad_s').value)
        self.steering_smoothing_alpha = float(self.get_parameter('steering_smoothing_alpha').value)
        self.min_control_speed_mps = float(self.get_parameter('min_control_speed_mps').value)
        self.cte_deadzone_m = float(self.get_parameter('cte_deadzone_m').value)
        self.lka_master_enabled = bool(self.get_parameter('lka_master_enabled').value)

    def parameters_callback_reconfigure_pid(self, params):
        result = SetParametersResult(successful=True)
        pid_changed = False
        for param in params:
            if param.name == 'stanley_k': self.stanley_k = param.value
            elif param.name == 'stanley_softening': self.stanley_softening = param.value
            elif param.name == 'pid_cte_kp': self.pid_cte_kp = param.value; pid_changed = True
            elif param.name == 'pid_cte_ki': self.pid_cte_ki = param.value; pid_changed = True
            elif param.name == 'pid_cte_kd': self.pid_cte_kd = param.value; pid_changed = True
            elif param.name == 'steering_smoothing_alpha': self.steering_smoothing_alpha = param.value
            elif param.name == 'cte_deadzone_m': self.cte_deadzone_m = param.value
        
        if pid_changed:
            self.pid_cte_controller = PIDController(
                kp=self.pid_cte_kp, ki=self.pid_cte_ki, kd=self.pid_cte_kd,
                integral_limit=self.pid_cte_integral_limit,
                output_limit=self.pid_cte_output_limit
            )
        return result

    def lka_enable_callback(self, msg: Bool):
        if self.lka_master_enabled != msg.data:
            self.lka_master_enabled = msg.data
            self.get_logger().info(f"LKA {'ENABLED' if self.lka_master_enabled else 'DISABLED'}")
            if not self.lka_master_enabled:
                self.pid_cte_controller.reset()
                self.last_steering_cmd_rad = 0.0
                self.smoothed_steering_cmd_rad = 0.0

    def pose_callback(self, msg: Pose):
        # Track vehicle movement for debugging
        self.last_vehicle_x = self.vehicle_x_map
        self.last_vehicle_y = self.vehicle_y_map
        
        self.vehicle_x_map = msg.position.x
        self.vehicle_y_map = msg.position.y
        self.vehicle_yaw_map = msg.orientation.z

    def vehicle_speed_callback(self, msg: Velocity):
        self.vehicle_speed_mps = msg.vehicle_velocity

    def lane_coordinates_callback(self, msg: String):
        try:
            lane_data = json.loads(msg.data)
            if 'center' in lane_data and isinstance(lane_data['center'], list):
                self.raw_path_waypoints = [(float(pt[0]), float(pt[1])) for pt in lane_data['center'] if len(pt) == 2]
                if len(self.raw_path_waypoints) > 1:
                    self.smoothed_path_waypoints = self._smooth_path(self.raw_path_waypoints)
                    self.is_reference_path_available = True
                    self.path_update_count += 1
                    self._publish_processed_path_viz()
                    
                    # Debug: Log path updates
                    if self.path_update_count % 20 == 0:
                        self.get_logger().info(f"🛣️ Path updated: {len(self.smoothed_path_waypoints)} points")
                else:
                    self.is_reference_path_available = False
            else:
                self.is_reference_path_available = False
        except Exception as e:
            self.get_logger().error(f"Lane coordinates error: {e}")
            self.is_reference_path_available = False

    def _smooth_path(self, waypoints):
        window_size = int(self.path_smoothing_window)
        if len(waypoints) < window_size or window_size < 2:
            return list(waypoints)
        if window_size % 2 == 0: window_size += 1
        
        half_window = window_size // 2
        path_x = np.array([wp[0] for wp in waypoints])
        path_y = np.array([wp[1] for wp in waypoints])

        smoothed_x = np.convolve(path_x, np.ones(window_size)/window_size, mode='same')
        smoothed_y = np.convolve(path_y, np.ones(window_size)/window_size, mode='same')
        
        if len(smoothed_x) > 0:
            smoothed_x[:half_window] = path_x[:half_window]
            smoothed_x[-half_window:] = path_x[-half_window:]
            smoothed_y[:half_window] = path_y[:half_window]
            smoothed_y[-half_window:] = path_y[-half_window:]
            
        return list(zip(smoothed_x, smoothed_y))

    def _publish_processed_path_viz(self):
        if not self.smoothed_path_waypoints: return
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.smoothed_path_waypoints:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = float(x), float(y), 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.processed_path_publisher.publish(path_msg)

    def _find_target_waypoint_index(self):
        if not self.smoothed_path_waypoints or len(self.smoothed_path_waypoints) < 2:
            return None, None

        # DYNAMIC LOOKAHEAD based on speed and CTE
        base_lookahead = self.lookahead_base_m + self.vehicle_speed_mps * self.lookahead_speed_factor
        
        # Reduce lookahead when CTE is large for more aggressive correction
        cte_factor = max(0.5, 1.0 - abs(self.current_cte) * 0.2)
        current_lookahead_dist = base_lookahead * cte_factor
        current_lookahead_dist = np.clip(current_lookahead_dist, self.min_lookahead_m, self.max_lookahead_m)

        veh_pos_np = np.array([self.vehicle_x_map, self.vehicle_y_map])
        path_np = np.array(self.smoothed_path_waypoints)
        
        if path_np.shape[0] == 0: return None, None

        # Find closest point
        distances_sq = np.sum((path_np - veh_pos_np)**2, axis=1)
        closest_idx = np.argmin(distances_sq)
        
        # Find target point at adaptive lookahead distance
        target_idx = closest_idx
        for i in range(closest_idx, len(self.smoothed_path_waypoints)):
            dist_to_point = math.sqrt((self.smoothed_path_waypoints[i][0] - veh_pos_np[0])**2 + 
                                     (self.smoothed_path_waypoints[i][1] - veh_pos_np[1])**2)
            if dist_to_point >= current_lookahead_dist:
                target_idx = i
                break
        else:
            target_idx = len(self.smoothed_path_waypoints) - 1
        
        # Ensure different indices for valid heading calculation
        if target_idx == closest_idx and target_idx < len(self.smoothed_path_waypoints) - 1:
            target_idx += 1
        elif target_idx == closest_idx and closest_idx > 0:
            closest_idx -= 1

        if closest_idx == target_idx:
            return None, None

        return closest_idx, target_idx

    def _calculate_stanley_control(self, closest_wp_tuple, target_wp_tuple, dt):
        path_p1 = closest_wp_tuple
        path_p2_for_heading = target_wp_tuple

        # Calculate path direction
        path_dx = path_p2_for_heading[0] - path_p1[0]
        path_dy = path_p2_for_heading[1] - path_p1[1]
        path_yaw = math.atan2(path_dy, path_dx)

        # Calculate CTE (Cross Track Error) with improved method
        vec_path1_to_vehicle_x = self.vehicle_x_map - path_p1[0]
        vec_path1_to_vehicle_y = self.vehicle_y_map - path_p1[1]
        
        path_segment_len = math.sqrt(path_dx**2 + path_dy**2)
        if path_segment_len < 0.01:
            cte_val = 0.0
        else:
            # Signed cross track error
            cte_val = (vec_path1_to_vehicle_x * path_dy - vec_path1_to_vehicle_y * path_dx) / path_segment_len

        # Apply deadzone to prevent small oscillations
        if abs(cte_val) < self.cte_deadzone_m:
            cte_val = 0.0

        # Calculate heading error
        heading_error = path_yaw - self.vehicle_yaw_map
        while heading_error > math.pi: heading_error -= 2 * math.pi
        while heading_error < -math.pi: heading_error += 2 * math.pi
        
        # Store for adaptive lookahead
        self.current_cte = cte_val
        self.current_heading_error = heading_error
        
        # RESPONSIVE STANLEY CONTROLLER
        current_vehicle_speed_mps = max(abs(self.vehicle_speed_mps), 0.1)
        
        # Stanley correction with speed-adaptive gain
        speed_factor = min(2.0, current_vehicle_speed_mps / 5.0)  # Reduce gain at very low speeds
        effective_k = self.stanley_k * speed_factor
        
        cte_correction_term = math.atan2(effective_k * cte_val, 
                                        current_vehicle_speed_mps + self.stanley_softening)

        stanley_steering_angle = heading_error + cte_correction_term

        # PID correction for fine-tuning
        pid_steering_correction = self.pid_cte_controller.compute(cte_val, dt)
        
        combined_steering_angle = stanley_steering_angle + pid_steering_correction

        # Log debug information
        if self._lka_log_counter % 20 == 0:  # Every 1 second at 20Hz
            self.get_logger().info(
                f"🎯 Stanley: CTE={cte_val:.3f}m, HdgErr={math.degrees(heading_error):.1f}°, "
                f"StanleyTerm={math.degrees(cte_correction_term):.1f}°, "
                f"PID={math.degrees(pid_steering_correction):.1f}°, "
                f"Final={math.degrees(combined_steering_angle):.1f}°"
            )

        return combined_steering_angle, cte_val, heading_error

    def control_loop(self):
        current_time = get_current_timestamp()
        dt = current_time - self.last_control_loop_time
        
        final_steering_cmd_rad = 0.0
        lka_active_for_control = False

        # Check if LKA should be active
        speed_ok = abs(self.vehicle_speed_mps) >= self.min_control_speed_mps
        
        if (self.lka_master_enabled and 
            self.is_reference_path_available and 
            len(self.smoothed_path_waypoints) >= 2 and
            speed_ok and
            dt > 0.001):
            
            closest_idx, target_idx = self._find_target_waypoint_index()

            if closest_idx is not None and target_idx is not None:
                closest_wp = self.smoothed_path_waypoints[closest_idx]
                target_wp = self.smoothed_path_waypoints[target_idx]

                calculated_steering_angle, cte, heading_error = self._calculate_stanley_control(closest_wp, target_wp, dt)
                
                # RESPONSIVE STEERING APPLICATION
                
                # 1. Apply absolute steering limit
                limited_steering = np.clip(calculated_steering_angle, 
                                         -self.max_lka_steering_wheel_angle_rad, 
                                         self.max_lka_steering_wheel_angle_rad)
                
                # 2. Apply rate limiting
                max_delta_steer = self.steering_rate_limit_rad_s * dt
                steering_change = limited_steering - self.last_steering_cmd_rad
                capped_steering_change = np.clip(steering_change, -max_delta_steer, max_delta_steer)
                rate_limited_steering = self.last_steering_cmd_rad + capped_steering_change
                
                # 3. Apply responsive smoothing (higher alpha = more responsive)
                self.smoothed_steering_cmd_rad = (self.steering_smoothing_alpha * rate_limited_steering + 
                                                 (1 - self.steering_smoothing_alpha) * self.smoothed_steering_cmd_rad)
                
                final_steering_cmd_rad = self.smoothed_steering_cmd_rad
                self.last_steering_cmd_rad = rate_limited_steering
                lka_active_for_control = True
                
            else:
                # Gradually return to neutral when path is invalid
                self.smoothed_steering_cmd_rad *= 0.8
                final_steering_cmd_rad = self.smoothed_steering_cmd_rad
                if abs(final_steering_cmd_rad) < 0.01:
                    final_steering_cmd_rad = 0.0
                    self.smoothed_steering_cmd_rad = 0.0
                    self.last_steering_cmd_rad = 0.0
        else:
            # LKA not active - gradually return to neutral
            self.smoothed_steering_cmd_rad *= 0.9
            final_steering_cmd_rad = self.smoothed_steering_cmd_rad
            if abs(final_steering_cmd_rad) < 0.01:
                final_steering_cmd_rad = 0.0
                self.smoothed_steering_cmd_rad = 0.0
                self.last_steering_cmd_rad = 0.0
                self.pid_cte_controller.reset()

        self.last_control_loop_time = current_time
        
        # Publish control command
        control_msg = Control()
        control_msg.steering = float(final_steering_cmd_rad)
        control_msg.throttle = 0.0
        control_msg.brake = 0.0
        control_msg.longswitch = 0
        control_msg.latswitch = 1 if lka_active_for_control else 0
        
        self.control_publisher.publish(control_msg)

        # Enhanced status logging
        self._lka_log_counter += 1
        if self._lka_log_counter % 100 == 0:  # Every 5 seconds
            status = "ACTIVE" if lka_active_for_control else "INACTIVE"
            path_status = "PATH_OK" if self.is_reference_path_available else "NO_PATH"
            speed_status = "SPEED_OK" if speed_ok else "TOO_SLOW"
            
            print(f"🛣️ LKA [{status}] {path_status} {speed_status}")
            print(f"   Vehicle: ({self.vehicle_x_map:.1f}, {self.vehicle_y_map:.1f}) @ {math.degrees(self.vehicle_yaw_map):.1f}°")
            print(f"   Speed: {self.vehicle_speed_mps:.1f}m/s | CTE: {self.current_cte:.2f}m")
            print(f"   Steering: {math.degrees(final_steering_cmd_rad):.1f}° | LatSw: {control_msg.latswitch}")
            print(f"   Path Updates: {self.path_update_count}")

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LaneKeepingAssistNode()
        print("🚀 Adaptive Stanley LKA started - will respond to lane changes...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: 
            print("🛑 LKA shutting down...")
    finally:
        if node and rclpy.ok():
            # Send neutral command
            final_ctrl = Control(steering=0.0, throttle=0.0, brake=0.0, longswitch=0, latswitch=0)
            node.control_publisher.publish(final_ctrl)
            node.get_logger().info("🔒 Neutral steering command sent")
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("✅ Adaptive LKA shutdown complete - Ashish480")

if __name__ == '__main__':
    main()