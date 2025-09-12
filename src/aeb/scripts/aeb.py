#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from enum import Enum

# Import required message types
from radar_msgs.msg import RadarTrack
from feedback.msg import Velocity
from vehiclecontrol.msg import Control
from rclpy.duration import Duration

# MODIFIED: Added an emergency state
class SimpleACCState(Enum):
    CRUISING = "cruising_to_target_speed"
    FOLLOWING = "following_lead_vehicle"
    EMERGENCY_BRAKE = "emergency_braking"

class SimpleACCController(Node):
    def __init__(self):
        super().__init__('simple_acc_controller')
        self.get_logger().info("Starting Simplified ACC Controller Node...")

        # === Core ACC Parameters ===
        self.TARGET_SPEED_KMPH = 30.0
        self.TARGET_SPEED_MS = self.TARGET_SPEED_KMPH / 3.6
        self.TARGET_FOLLOW_DISTANCE = 4.0

        # === NEW: Emergency Braking Parameters ===
        # The forward distance that defines the emergency "danger zone"
        self.OBSTACLE_FORWARD_DISTANCE = 3.0
        # The lateral distance (half-width) that defines the emergency "danger zone"
        self.OBSTACLE_LATERAL_DISTANCE = 1.0

        # === Control Gains ===
        self.KP_SPEED = 0.4
        self.KP_GAP = 0.15

        # === State Variables ===
        self.current_speed_ms = 0.0
        self.leading_distance = float('inf')
        self.acc_state = SimpleACCState.CRUISING
        # NEW: Flag to indicate if an obstacle is in the immediate danger zone
        self.obstacle_in_danger_zone = False

        # === ROS2 Subscribers and Publishers ===
        self.radar_sub = self.create_subscription(
            RadarTrack, 'RadarObjects', self.radar_callback, 10)
        self.velocity_sub = self.create_subscription(
            Velocity, 'VehicleSpeed', self.velocity_callback, 10)
        self.control_pub = self.create_publisher(Control, '/vehicle_control', 10)

        # Main control loop timer (runs 50 times per second)
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info("Simplified ACC Controller initialized.")

    # --- THIS IS YOUR NEW RADAR CALLBACK, INTEGRATED WITH THE ACC LOGIC ---
    def radar_callback(self, msg: RadarTrack):
        """
        Processes radar data for two purposes:
        1. Checks if an object is in the emergency "danger zone".
        2. Finds the closest in-path vehicle for standard ACC following.
        """
        # Assume the message fields are x_distance and y_distance as per your snippet
        # If your message uses `position.x`, change `msg.x_distance` to `msg.position.x`
        forward_dist = msg.x_distance
        lateral_dist = msg.y_distance

        # --- Logic 1: Emergency Zone Check ---
        is_too_close = (0 < forward_dist <= self.OBSTACLE_FORWARD_DISTANCE)
        is_in_danger_path = (abs(lateral_dist) <= self.OBSTACLE_LATERAL_DISTANCE)

        if is_too_close and is_in_danger_path:
            self.obstacle_in_danger_zone = True
        else:
            # If the current object is not in the danger zone, we can't immediately
            # say the zone is clear. We reset this flag in the main loop after
            # checking all objects. For a single object message, we can reset here.
            self.obstacle_in_danger_zone = False

        # --- Logic 2: Standard ACC Lead Vehicle Tracking ---
        is_in_front = forward_dist > 0
        is_in_follow_path = abs(lateral_dist) < 1.5 # Standard lane width check

        if is_in_front and is_in_follow_path:
            # This is a valid lead vehicle, update the distance for ACC
            self.leading_distance = forward_dist
        else:
            # This object is not a valid lead vehicle, reset to infinity
            self.leading_distance = float('inf')

    def velocity_callback(self, msg: Velocity):
        """
        Updates the vehicle's current speed from feedback.
        """
        self.current_speed_ms = msg.vehicle_velocity

    def update_acc_state(self):
        """
        Switches between states based on emergency flag and leading distance.
        The emergency state has the highest priority.
        """
        previous_state = self.acc_state

        if self.obstacle_in_danger_zone:
            self.acc_state = SimpleACCState.EMERGENCY_BRAKE
        elif self.leading_distance != float('inf'):
            self.acc_state = SimpleACCState.FOLLOWING
        else:
            self.acc_state = SimpleACCState.CRUISING

        if self.acc_state != previous_state:
            self.get_logger().info(f"ACC State Change: {previous_state.name} -> {self.acc_state.name}")

    def calculate_cruising_throttle(self):
        """
        Calculates throttle to reach the target speed.
        """
        speed_error = self.TARGET_SPEED_MS - self.current_speed_ms
        throttle = self.KP_SPEED * speed_error
        throttle = max(0.0, min(throttle, 0.7))
        return throttle, 0.0

    def calculate_following_control(self):
        """
        Calculates throttle and brake to maintain the target follow distance.
        """
        gap_error = self.leading_distance - self.TARGET_FOLLOW_DISTANCE
        control_adjustment = self.KP_GAP * gap_error

        if control_adjustment > 0:
            throttle = min(control_adjustment, 1.0)
            brake = 0.0
        else:
            throttle = 0.0
            brake = min(abs(control_adjustment), 1.0)

        return throttle, brake

    def control_loop(self):
        """
        The main loop that runs continuously to control the vehicle.
        """
        self.update_acc_state()

        throttle, brake = 0.0, 0.0

        # MODIFIED: The control logic now checks for the emergency state first.
        if self.acc_state == SimpleACCState.EMERGENCY_BRAKE:
            throttle = 0.0
            brake = 1.0  # Apply full brakes
        elif self.acc_state == SimpleACCState.CRUISING:
            throttle, brake = self.calculate_cruising_throttle()
        elif self.acc_state == SimpleACCState.FOLLOWING:
            throttle, brake = self.calculate_following_control()

        # Create and publish the final control message
        control_msg = Control()
        control_msg.throttle = throttle
        control_msg.brake = brake
        control_msg.steering = 0.0
        control_msg.longswitch = 1
        control_msg.latswitch = 0
        self.control_pub.publish(control_msg)

        # Log a summary of the status periodically
        if not hasattr(self, '_summary_log_counter'): self._summary_log_counter = 0
        self._summary_log_counter = (self._summary_log_counter + 1)
        if self._summary_log_counter % 50 == 0:
            lead_info = f"Lead @ {self.leading_distance:.1f}m" if self.leading_distance != float('inf') else "No Lead"
            self.get_logger().info(
                f"[{self.acc_state.name}] Speed: {self.current_speed_ms*3.6:.1f} km/h | "
                f"Cmd -> T:{throttle:.2f} B:{brake:.2f} | {lead_info}"
            )

def main(args=None):
    rclpy.init(args=args)
    acc_controller_node = None
    try:
        acc_controller_node = SimpleACCController()
        rclpy.spin(acc_controller_node)
    except KeyboardInterrupt:
        if acc_controller_node: acc_controller_node.get_logger().info("Shutting down...")
    finally:
        if acc_controller_node:
            final_control = Control(throttle=0.0, brake=1.0, longswitch=1)
            acc_controller_node.control_pub.publish(final_control)
            acc_controller_node.get_logger().info("Sent final safe brake command.")
            time.sleep(0.1)
            acc_controller_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
