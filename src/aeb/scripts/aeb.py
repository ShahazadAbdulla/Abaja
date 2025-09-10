#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from enum import Enum

from vehiclecontrol.msg import Control
from radar_msgs.msg import RadarTrack
from feedback.msg import Velocity

# State machine updated to reflect the new braking action
class TestState(Enum):
    IDLE = "idle_waiting_for_feedback"
    WAITING_DELAY = "feedback_received_waiting_2s"
    RAMPING = "ramping_throttle_step"
    HOLDING_MAX = "holding_max_throttle_indefinitely"
    EMERGENCY_BRAKING = "emergency_braking" # CHANGED

class AccelerationTestNode(Node):
    def __init__(self):
        super().__init__('acceleration_test_node')
        self.get_logger().info("Starting Final Feedback-Driven Acceleration Test (with Braking)...")

        # === TEST PARAMETERS ===
        self.THROTTLE_START_VALUE = 0.1
        self.THROTTLE_END_VALUE = 1.0
        self.THROTTLE_STEP = 0.1
        self.DELAY_AFTER_FEEDBACK_SECONDS = 2.0

        # === RADAR SAFETY PARAMETERS ===
        self.OBSTACLE_FORWARD_DISTANCE = 3.0
        self.OBSTACLE_LATERAL_DISTANCE = 0.5
        self.OBSTACLE_CLEAR_TIMEOUT = 0.2

        # === State variables ===
        self.current_speed = 0.0
        self.current_state = TestState.IDLE
        self.current_throttle_level = 0.0
        self.feedback_received_time = None
        self.obstacle_in_danger_zone = False
        self.last_obstacle_time = None

        # Control outputs
        self.throttle_output = 0.0
        self.brake_output = 0.0 # Brake is now actively used

        # === ROS2 Publishers and Subscribers ===
        self.control_pub = self.create_publisher(Control, '/vehicle_control', 10)
        self.radar_sub = self.create_subscription(
            RadarTrack, 'RadarObjects', self.radar_callback, 10)
        self.velocity_sub = self.create_subscription(
            Velocity, 'VehicleSpeed', self.velocity_callback, 10)
        
        # Main control loop timer
        self.control_timer = self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info("Node initialized. Emergency action is now BRAKE = 1.0.")

    def velocity_callback(self, msg: Velocity):
        """
        Receives velocity feedback, which signals the start of the 2-second wait period.
        """
        self.current_speed = msg.vehicle_velocity
        
        if self.current_state in [TestState.IDLE, TestState.RAMPING]:
            self.feedback_received_time = self.get_clock().now()
            self.current_state = TestState.WAITING_DELAY
            self.get_logger().info(f"Feedback received. Starting {self.DELAY_AFTER_FEEDBACK_SECONDS}s timer before next ramp.")

    def radar_callback(self, msg: RadarTrack):
        """
        Checks each radar object and triggers the emergency state if it's in the safety box.
        """
        is_too_close = (0 < msg.x_distance <= self.OBSTACLE_FORWARD_DISTANCE)
        is_in_path = (abs(msg.y_distance) <= self.OBSTACLE_LATERAL_DISTANCE)

        if is_too_close and is_in_path:
            self.obstacle_in_danger_zone = True
            self.last_obstacle_time = self.get_clock().now()

    def reset_test_procedure(self):
        """Resets the entire test to its initial state."""
        self.get_logger().info("Obstacle cleared. Resetting test. Waiting for feedback to begin.")
        self.current_throttle_level = 0.0
        self.feedback_received_time = None
        self.current_state = TestState.IDLE

    def control_loop(self):
        """Main control loop where all state logic is handled."""
        previous_state = self.current_state
        
        # --- 1. Handle Obstacle Detection and Clearing (Highest Priority) ---
        if self.obstacle_in_danger_zone and self.last_obstacle_time:
            time_since_obs = (self.get_clock().now() - self.last_obstacle_time).nanoseconds / 1e9
            if time_since_obs > self.OBSTACLE_CLEAR_TIMEOUT:
                self.obstacle_in_danger_zone = False
                self.last_obstacle_time = None
        
        if self.obstacle_in_danger_zone:
            self.current_state = TestState.EMERGENCY_BRAKING # CHANGED
        elif not self.obstacle_in_danger_zone and previous_state == TestState.EMERGENCY_BRAKING: # CHANGED
            self.reset_test_procedure()
            
        # --- 2. State Machine Logic ---
        # Default to no brake unless in emergency state
        self.brake_output = 0.0

        if self.current_state == TestState.IDLE:
            self.throttle_output = 0.0
            
        elif self.current_state == TestState.EMERGENCY_BRAKING: # CHANGED
            self.throttle_output = 0.0
            self.brake_output = 1.0 # CHANGED: Apply full brake
            if previous_state != TestState.EMERGENCY_BRAKING:
                 self.get_logger().warn("Obstacle Detected! Applying full BRAKE (1.0).") # CHANGED log message

        elif self.current_state == TestState.WAITING_DELAY:
            if self.feedback_received_time:
                elapsed_time = (self.get_clock().now() - self.feedback_received_time).nanoseconds / 1e9
                if elapsed_time >= self.DELAY_AFTER_FEEDBACK_SECONDS:
                    self.current_state = TestState.RAMPING
            
        elif self.current_state == TestState.RAMPING:
            if self.current_throttle_level < self.THROTTLE_START_VALUE:
                self.current_throttle_level = self.THROTTLE_START_VALUE
            else:
                self.current_throttle_level += self.THROTTLE_STEP

            self.current_throttle_level = min(self.current_throttle_level, self.THROTTLE_END_VALUE)
            self.get_logger().info(f"2s delay complete. Ramping to throttle: {self.current_throttle_level:.1f}")
            self.get_logger().info("Now waiting for next velocity feedback...")
            
            self.throttle_output = self.current_throttle_level
            
            if self.current_throttle_level >= self.THROTTLE_END_VALUE:
                self.current_state = TestState.HOLDING_MAX
                self.get_logger().info("Reached max throttle. Holding indefinitely.")

        elif self.current_state == TestState.HOLDING_MAX:
            self.throttle_output = self.THROTTLE_END_VALUE
        
        # --- 3. Publish Control Message ---
        control_msg = Control()
        control_msg.throttle = float(self.throttle_output)
        control_msg.brake = float(self.brake_output) # CHANGED: Now uses the brake_output variable
        control_msg.steering = 0.0
        control_msg.longswitch = 1
        control_msg.latswitch = 0
        self.control_pub.publish(control_msg)
        
        # --- 4. Periodic Logging ---
        if not hasattr(self, 'last_log_time'): self.last_log_time = 0
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_log_time > 1.0:
            self.last_log_time = current_time
            self.get_logger().info(
                f"State: {self.current_state.value} | "
                f"Throttle: {self.throttle_output:.2f} | "
                f"Brake: {self.brake_output:.2f} | " # ADDED Brake to log
                f"Speed: {self.current_speed*3.6:.1f} km/h"
            )

def main(args=None):
    rclpy.init(args=args)
    test_node = None
    try:
        test_node = AccelerationTestNode()
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        if test_node: test_node.get_logger().info("Node shutting down...")
    finally:
        if test_node:
            # CHANGED: Send a final safe BRAKE command on shutdown
            final_control = Control(throttle=0.0, brake=1.0, longswitch=1)
            test_node.control_pub.publish(final_control)
            test_node.get_logger().info("Sent final safe brake command.")
            time.sleep(0.1)
            test_node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()