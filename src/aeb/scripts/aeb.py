#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from enum import Enum

from vehiclecontrol.msg import Control
from radar_msgs.msg import RadarTrack
from feedback.msg import Velocity

# The state machine is updated for the new logic
class TestState(Enum):
    IDLE = "idle_waiting_for_start"
    RAMPING = "ramping_on_feedback"
    HOLDING_MAX = "holding_max_throttle_indefinitely"
    EMERGENCY_THROTTLE_CUT = "emergency_throttle_cut"

class AccelerationTestNode(Node):
    def __init__(self):
        super().__init__('acceleration_test_node')
        self.get_logger().info("Starting Feedback-Driven Acceleration Test Node...")

        # === TEST PARAMETERS ===
        self.THROTTLE_START_VALUE = 0.1
        self.THROTTLE_END_VALUE = 1.0
        self.THROTTLE_STEP = 0.1

        # === RADAR SAFETY PARAMETERS ===
        self.OBSTACLE_FORWARD_DISTANCE = 6.0
        self.OBSTACLE_LATERAL_DISTANCE = 1.5
        self.OBSTACLE_CLEAR_TIMEOUT = 0.2

        # === State variables ===
        self.current_speed = 0.0
        self.current_state = TestState.IDLE
        self.current_throttle_level = 0.0
        
        # This flag controls the feedback loop for ramping
        self.waiting_for_velocity_feedback = True
        
        self.obstacle_in_danger_zone = False
        self.last_obstacle_time = None

        # Control outputs
        self.throttle_output = 0.0

        # === ROS2 Publishers and Subscribers ===
        self.control_pub = self.create_publisher(Control, '/vehicle_control', 10)
        self.radar_sub = self.create_subscription(
            RadarTrack, 'RadarObjects', self.radar_callback, 10)
        self.velocity_sub = self.create_subscription(
            Velocity, 'VehicleSpeed', self.velocity_callback, 10)
        
        # Control loop timer
        self.control_timer = self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info("Node initialized. Waiting for first velocity feedback to start test.")

    def velocity_callback(self, msg: Velocity):
        """
        Receives velocity feedback. This is the trigger to proceed with the next ramp step.
        """
        self.current_speed = msg.vehicle_velocity
        # If we were in IDLE state, receiving feedback starts the test.
        if self.current_state == TestState.IDLE:
            self.current_state = TestState.RAMPING
        
        # Signal that we have received feedback and can ramp up again.
        self.waiting_for_velocity_feedback = False

    def radar_callback(self, msg: RadarTrack):
        """
        Checks each radar object. If it's in the safety box, trigger the emergency state.
        """
        is_too_close = (0 < msg.x_distance <= self.OBSTACLE_FORWARD_DISTANCE)
        is_in_path = (abs(msg.y_distance) <= self.OBSTACLE_LATERAL_DISTANCE)

        if is_too_close and is_in_path:
            self.obstacle_in_danger_zone = True
            self.last_obstacle_time = self.get_clock().now()

    def reset_test_procedure(self):
        """Resets the test to its initial ramping state."""
        self.get_logger().info("Obstacle cleared. Resetting test and starting ramp-up again.")
        self.current_throttle_level = 0.0
        self.waiting_for_velocity_feedback = True # Wait for new feedback to begin
        self.current_state = TestState.RAMPING

    def control_loop(self):
        """Main control loop executed at 50Hz."""
        previous_state = self.current_state
        
        # --- 1. Handle Obstacle Detection and Clearing ---
        # Check if the obstacle timeout has passed
        if self.obstacle_in_danger_zone and self.last_obstacle_time:
            time_since_obs = (self.get_clock().now() - self.last_obstacle_time).nanoseconds / 1e9
            if time_since_obs > self.OBSTACLE_CLEAR_TIMEOUT:
                self.obstacle_in_danger_zone = False
                self.last_obstacle_time = None
        
        # If an obstacle is present, it forces the state to emergency cut.
        if self.obstacle_in_danger_zone:
            self.current_state = TestState.EMERGENCY_THROTTLE_CUT
        # If there's NO obstacle AND the previous state was an emergency, reset the test.
        elif not self.obstacle_in_danger_zone and previous_state == TestState.EMERGENCY_THROTTLE_CUT:
            self.reset_test_procedure()
            
        # --- 2. Determine State and Calculate Throttle ---
        if self.current_state == TestState.IDLE:
            self.throttle_output = 0.0
            
        elif self.current_state == TestState.EMERGENCY_THROTTLE_CUT:
            self.throttle_output = 0.0
            if previous_state != TestState.EMERGENCY_THROTTLE_CUT:
                 self.get_logger().warn("Obstacle Detected! Cutting throttle to ZERO.")

        elif self.current_state == TestState.RAMPING:
            # Only ramp up if we have received velocity feedback since the last step
            if not self.waiting_for_velocity_feedback:
                if self.current_throttle_level < self.THROTTLE_START_VALUE:
                    self.current_throttle_level = self.THROTTLE_START_VALUE
                else:
                    self.current_throttle_level += self.THROTTLE_STEP

                self.current_throttle_level = min(self.current_throttle_level, self.THROTTLE_END_VALUE)
                self.get_logger().info(f"Feedback received. Ramping to throttle: {self.current_throttle_level:.1f}")

                # Now, wait for the next feedback message before ramping again
                self.waiting_for_velocity_feedback = True
            
            self.throttle_output = self.current_throttle_level
            
            # Transition to HOLDING_MAX if we've reached the end value
            if self.current_throttle_level >= self.THROTTLE_END_VALUE:
                self.current_state = TestState.HOLDING_MAX
                self.get_logger().info("Reached max throttle. Holding indefinitely.")

        elif self.current_state == TestState.HOLDING_MAX:
            self.throttle_output = self.THROTTLE_END_VALUE
        
        # --- 3. Publish Control Message ---
        control_msg = Control()
        control_msg.throttle = float(self.throttle_output)
        control_msg.brake = 0.0  # Brake is always 0
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
                f"Throttle Cmd: {self.throttle_output:.2f} | "
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
            final_control = Control(throttle=0.0, brake=0.0, longswitch=1)
            test_node.control_pub.publish(final_control)
            test_node.get_logger().info("Sent final zero-throttle command.")
            time.sleep(0.1)
            test_node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()