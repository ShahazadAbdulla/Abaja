#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import json
import numpy as np
import time
from collections import deque

class PIDController:
    """Simple PID Controller for Lane Keeping Assist"""

    def __init__(self, kp=20.0, ki=0.1, kd=5.0, output_limits=(0, 270)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits

        # PID state variables
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None

        # For debugging
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0

    def update(self, error, dt):
        """Update PID controller with current error and time delta"""
        if dt <= 0:
            return 135.0  # Center value if no time elapsed

        # Proportional term
        self.p_term = self.kp * error

        # Integral term
        self.integral += error * dt
        self.i_term = self.ki * self.integral

        # Derivative term
        derivative = (error - self.prev_error) / dt
        self.d_term = self.kd * derivative

        # Calculate PID output
        pid_output = self.p_term + self.i_term + self.d_term

        # Convert to steering command (center around 135 degrees)
        steering_cmd = 135.0 - pid_output  # Negative error = left deviation = steer right

        # Clip to output limits
        steering_cmd = np.clip(steering_cmd, self.output_limits[0], self.output_limits[1])

        # Update for next iteration
        self.prev_error = error

        return steering_cmd

    def reset(self):
        """Reset PID controller state"""
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None

class LaneKeepingAssistNode(Node):
    """ROS2 Node for Lane Keeping Assist that works with lanedetection.py output"""

    def __init__(self):
        super().__init__('lane_keeping_assist_node')

        # Declare parameters
        self.declare_parameter('kp', 20.0)
        self.declare_parameter('ki', 0.1)
        self.declare_parameter('kd', 5.0)
        self.declare_parameter('angle_weight', 0.7)
        self.declare_parameter('pid_weight', 0.3)
        self.declare_parameter('pixel_to_meter', 0.01)  # Conversion from pixels to meters
        self.declare_parameter('frame_width', 1280)
        self.declare_parameter('frame_height', 720)

        # Get parameters
        kp = self.get_parameter('kp').get_parameter_value().double_value
        ki = self.get_parameter('ki').get_parameter_value().double_value
        kd = self.get_parameter('kd').get_parameter_value().double_value
        self.angle_weight = self.get_parameter('angle_weight').get_parameter_value().double_value
        self.pid_weight = self.get_parameter('pid_weight').get_parameter_value().double_value
        self.pixel_to_meter = self.get_parameter('pixel_to_meter').get_parameter_value().double_value
        self.frame_width = self.get_parameter('frame_width').get_parameter_value().integer_value
        self.frame_height = self.get_parameter('frame_height').get_parameter_value().integer_value

        # Initialize PID Controller
        self.pid = PIDController(kp=kp, ki=ki, kd=kd)

        # Smoothing for reference line (same as lka_pid.py)
        self.recent_slopes = deque(maxlen=10)
        self.recent_intercepts = deque(maxlen=10)

        # Timing
        self.prev_time = None

        # Statistics
        self.frame_count = 0
        self.valid_data_count = 0

        # Create subscriber to lane coordinates
        self.lane_coords_sub = self.create_subscription(
            String,
            '/lane_coordinates',
            self.lane_coordinates_callback,
            10
        )

        # Create publisher for steering commands
        self.steering_pub = self.create_publisher(
            Float32,
            '/steering_command',
            10
        )

        # Create publisher for debug information
        self.debug_pub = self.create_publisher(
            String,
            '/lka_debug',
            10
        )

        self.get_logger().info('🚗 Lane Keeping Assist Node initialized')
        self.get_logger().info(f'🎛️  PID Parameters: Kp={kp}, Ki={ki}, Kd={kd}')
        self.get_logger().info(f'⚖️  Fusion weights: Angle={self.angle_weight}, PID={self.pid_weight}')
        self.get_logger().info(f'📏 Pixel conversion: {self.pixel_to_meter} m/px')
        self.get_logger().info('📡 Subscribing to /lane_coordinates topic...')

    def calculate_lane_angle(self, center_points):
        """Calculate the angle of the lane relative to vehicle direction"""
        if len(center_points) < 4:
            return 0.0

        # Convert to numpy array for easier manipulation
        points_array = np.array(center_points)
        x_coords = points_array[:, 0]
        y_coords = points_array[:, 1]

        # Use middle portion of points to calculate direction
        n = len(points_array) // 2
        dx = x_coords[-1] - x_coords[-n]  # Change in X over half the points
        dy = y_coords[-1] - y_coords[-n]  # Change in Y over half the points

        # Create lane direction vector
        lane_vec = np.array([dx, -dy], dtype=float)  # -dy because y increases downward
        norm = np.linalg.norm(lane_vec)
        if norm < 1e-6:
            return 0.0
        lane_vec /= norm

        # Reference vector (straight ahead)
        ref_vec = np.array([0, -1], dtype=float)  # Points upward (vehicle direction)

        # Calculate angle between vectors
        angle_rad = np.arctan2(np.cross(ref_vec, lane_vec), np.dot(ref_vec, lane_vec))
        return np.degrees(angle_rad)

    def map_steering_angle(self, lane_angle, max_input_angle=270.0):
        """Map lane angle to steering command (same as lka_pid.py)"""
        CENTER_CMD = 135.0

        if lane_angle < 0:
            return np.interp(abs(lane_angle), [0, max_input_angle], [CENTER_CMD, 0])
        else:
            return np.interp(lane_angle, [0, max_input_angle], [CENTER_CMD, 270])

    def calculate_reference_line(self, lane_data):
        """Calculate reference line from lanedetection.py output"""
        try:
            # Priority order: center > center_line > fallback to single lanes
            center_points = None
            strategy_used = lane_data.get('strategy', 'unknown')

            # First, try to get the center line from lanedetection.py
            if 'center' in lane_data and lane_data['center']:
                center_points = lane_data['center']
                self.get_logger().debug(f'Using center field with {len(center_points)} points')
            elif 'center_line' in lane_data and lane_data['center_line']:
                center_points = lane_data['center_line']
                self.get_logger().debug(f'Using center_line field with {len(center_points)} points')

            if not center_points or len(center_points) < 2:
                self.get_logger().warn(f'No valid center points found. Available keys: {list(lane_data.keys())}')
                return None, None, None, None

            # Convert to numpy array and validate format
            center_array = np.array(center_points)
            if center_array.ndim != 2 or center_array.shape[1] != 2:
                self.get_logger().warn(f'Invalid center points format: {center_array.shape}')
                return None, None, None, None

            x_center = center_array[:, 0]
            y_center = center_array[:, 1]

            # Use bottom portion for more stable calculations (same as lka_pid.py)
            valid_indices = y_center >= self.frame_height * 0.65
            if not np.any(valid_indices):
                valid_indices = np.ones(len(y_center), dtype=bool)  # Use all if none in bottom

            x_center_valid = x_center[valid_indices]
            y_center_valid = y_center[valid_indices]

            if len(x_center_valid) < 2:
                return None, None, None, None

            # Calculate vehicle center position
            vehicle_center_x = self.frame_width // 2

            # Calculate lateral deviation (using bottom-most point)
            lateral_dev_px = x_center_valid[-1] - vehicle_center_x
            lateral_dev_m = lateral_dev_px * self.pixel_to_meter

            # Calculate lane angle using the center line
            raw_lane_angle = self.calculate_lane_angle(list(zip(x_center_valid, y_center_valid)))
            mapped_lane_angle = self.map_steering_angle(raw_lane_angle)

            # Update smoothed reference line (same as lka_pid.py)
            if len(x_center_valid) >= 2:
                # Fit linear model for smoothing
                n_bottom = max(1, len(y_center_valid) // 5)
                x_bottom = x_center_valid[-n_bottom:]
                y_bottom = y_center_valid[-n_bottom:]

                current_coeffs = np.polyfit(y_bottom, x_bottom, 1)
                self.recent_slopes.append(current_coeffs[0])
                self.recent_intercepts.append(current_coeffs[1])

            return lateral_dev_m, raw_lane_angle, mapped_lane_angle, center_points

        except Exception as e:
            self.get_logger().error(f'Error calculating reference line: {str(e)}')
            return None, None, None, None

    def lane_coordinates_callback(self, msg):
        """Process incoming lane coordinates and publish steering command"""
        try:
            # Parse JSON data
            lane_data = json.loads(msg.data)
            self.frame_count += 1

            # Log first few messages to debug format
            if self.frame_count <= 3:
                self.get_logger().info(f'📨 Received lane data keys: {list(lane_data.keys())}')
                if 'center' in lane_data:
                    center_points = lane_data['center']
                    self.get_logger().info(f'🎯 Center points: {len(center_points)} available')
                if 'strategy' in lane_data:
                    self.get_logger().info(f'🛣️  Strategy: {lane_data["strategy"]}')
                if 'status' in lane_data:
                    self.get_logger().info(f'📊 Status: {lane_data["status"]}')

            # Calculate reference line and deviations
            lateral_dev_m, raw_lane_angle, mapped_lane_angle, center_points = self.calculate_reference_line(lane_data)

            # Default values
            pid_output = 135.0
            final_steering_cmd = 135.0

            if lateral_dev_m is not None and mapped_lane_angle is not None:
                self.valid_data_count += 1

                # PID Controller
                current_time = time.time()
                if self.prev_time is not None:
                    dt = current_time - self.prev_time
                    pid_output = self.pid.update(lateral_dev_m, dt)

                    # Fuse mapped angle with PID output (same as lka_pid.py)
                    final_steering_cmd = (self.angle_weight * mapped_lane_angle + 
                                        self.pid_weight * pid_output)
                    final_steering_cmd = np.clip(final_steering_cmd, 0, 270)
                else:
                    final_steering_cmd = mapped_lane_angle

                self.prev_time = current_time
            else:
                # Fallback to center steering
                if self.frame_count % 100 == 0:  # Reduce warning frequency
                    self.get_logger().warn('⚠️  Using fallback center steering - no valid center line')
                final_steering_cmd = 135.0

            # Publish steering command
            steering_msg = Float32()
            steering_msg.data = float(final_steering_cmd)
            self.steering_pub.publish(steering_msg)

            # Publish debug information
            debug_data = {
                'frame_count': self.frame_count,
                'valid_data_count': self.valid_data_count,
                'strategy': lane_data.get('strategy', 'unknown'),
                'status': lane_data.get('status', 'unknown'),
                'center_points_count': len(center_points) if center_points else 0,
                'lateral_deviation_m': lateral_dev_m if lateral_dev_m is not None else 0.0,
                'raw_lane_angle': raw_lane_angle if raw_lane_angle is not None else 0.0,
                'mapped_lane_angle': mapped_lane_angle if mapped_lane_angle is not None else 135.0,
                'pid_output': float(pid_output),
                'final_steering_cmd': float(final_steering_cmd),
                'p_term': float(self.pid.p_term),
                'i_term': float(self.pid.i_term),
                'd_term': float(self.pid.d_term),
                'smoothed_slope': float(np.mean(self.recent_slopes)) if self.recent_slopes else 0.0,
                'smoothed_intercept': float(np.mean(self.recent_intercepts)) if self.recent_intercepts else 0.0,
                'data_valid': lateral_dev_m is not None
            }

            debug_msg = String()
            debug_msg.data = json.dumps(debug_data)
            self.debug_pub.publish(debug_msg)

            # Log periodic updates with safe formatting
            if self.frame_count % 30 == 0:  # Every 30 frames (~1 second at 30fps)
                valid_rate = (self.valid_data_count / self.frame_count) * 100
                strategy = lane_data.get('strategy', 'N/A')
                status = lane_data.get('status', 'N/A')
                center_count = len(center_points) if center_points else 0

                self.get_logger().info(
                    f'🚗 Frame {self.frame_count:4d} | Valid: {valid_rate:5.1f}% | '
                    f'Strategy: {strategy} | Center pts: {center_count} | '
                    f'Lat Dev: {lateral_dev_m:.3f}m | Steering: {final_steering_cmd:5.1f}°'
                )

        except json.JSONDecodeError as e:
            self.get_logger().error(f'❌ Failed to parse lane coordinates JSON: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'❌ Error in lane coordinates callback: {str(e)}')

def main(args=None):
    rclpy.init(args=args)

    lka_node = None
    try:
        lka_node = LaneKeepingAssistNode()
        rclpy.spin(lka_node)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down Lane Keeping Assist Node...")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if lka_node is not None:
            lka_node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass  # Ignore shutdown errors

if __name__ == '__main__':
    main()

