#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import time
from enum import Enum

from vehiclecontrol.msg import Control

# Simplified state machine for the acceleration test
class TestState(Enum):
    RAMPING = "ramping_throttle"
    HOLDING_MAX = "holding_max_throttle"
    MOTOR_OFF = "motor_off"

class AccelerationTestNode(Node):
    def __init__(self):
        super().__init__('acceleration_test_node')
        self.get_logger().info("Starting Staged Acceleration Test Node...")

        # === TEST PARAMETERS ===
        self.THROTTLE_START_VALUE = 0.1
        self.THROTTLE_END_VALUE = 1.0
        self.THROTTLE_STEP = 0.1
        self.RAMP_INTERVAL_SECONDS = 2.0  # Time to hold each throttle step
        self.MAX_HOLD_DURATION = 4.0  # Time to hold max throttle before turning off motor

        # === State variables ===
        self.current_speed = 0.0
        self.current_state = TestState.RAMPING
        self.current_throttle_level = 0.0
        self.last_ramp_time = self.get_clock().now()
        self.max_throttle_start_time = None  # Track when we started holding max throttle

        # Control outputs
        self.throttle_output = 0.0
        self.brake_output = 0.0  # Brake is unused, will always be 0.0
        self.steering_output = 0.0

        # Control publisher
        self.control_pub = self.create_publisher(Control, '/vehicle_control', 10)
        
        # Control loop timer (50Hz = 0.02s)
        self.control_timer = self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info("Staged Acceleration Test Node initialized successfully.")

    def determine_vehicle_state(self):
        """Determine the current state based on test progress."""
        # If we're already in motor off state, stay there
        if self.current_state == TestState.MOTOR_OFF:
            return TestState.MOTOR_OFF
        
        # If the throttle has reached its max value, check timing for motor off
        if self.current_throttle_level >= self.THROTTLE_END_VALUE:
            # If we just reached max throttle, record the time
            if self.current_state != TestState.HOLDING_MAX:
                self.max_throttle_start_time = self.get_clock().now()
                self.get_logger().info("Reached maximum throttle. Starting 4-second countdown to motor off.")
                return TestState.HOLDING_MAX
            
            # Check if we've been holding max throttle for 4 seconds
            if self.max_throttle_start_time is not None:
                now = self.get_clock().now()
                time_at_max = (now - self.max_throttle_start_time).nanoseconds / 1e9
                if time_at_max >= self.MAX_HOLD_DURATION:
                    self.get_logger().info("4 seconds elapsed. Turning motor off (throttle = 0.0).")
                    return TestState.MOTOR_OFF
            
            return TestState.HOLDING_MAX

        # Default state is to continue ramping up.
        return TestState.RAMPING

    def calculate_control_outputs(self):
        """Calculate throttle based on the current state of the test."""
        # The brake is unused in this logic.
        self.brake_output = 0.0
        
        if self.current_state == TestState.RAMPING:
            # Check if it's time to increase the throttle
            now = self.get_clock().now()
            time_since_last_ramp = (now - self.last_ramp_time).nanoseconds / 1e9

            if time_since_last_ramp >= self.RAMP_INTERVAL_SECONDS:
                # If starting from zero, begin with the start value. Otherwise, increment.
                if self.current_throttle_level < self.THROTTLE_START_VALUE:
                    self.current_throttle_level = self.THROTTLE_START_VALUE
                else:
                    self.current_throttle_level += self.THROTTLE_STEP
                
                # Ensure the throttle doesn't exceed the end value
                self.current_throttle_level = min(self.current_throttle_level, self.THROTTLE_END_VALUE)
                
                self.get_logger().info(f"Ramping up: Setting throttle to {self.current_throttle_level:.1f}")
                self.last_ramp_time = now # Reset the timer for the next interval
            
            self.throttle_output = self.current_throttle_level

        elif self.current_state == TestState.HOLDING_MAX:
            # The ramp is complete, just hold the max throttle value.
            self.throttle_output = self.THROTTLE_END_VALUE
            
        elif self.current_state == TestState.MOTOR_OFF:
            # Motor is turned off, set throttle to zero
            self.throttle_output = 0.0

    def control_loop(self):
        """Main control loop executed at 50Hz."""
        # 1. Determine the vehicle's current state (Ramping, Holding, or Stop)
        self.current_state = self.determine_vehicle_state()
        
        # 2. Calculate the appropriate throttle value based on the state
        self.calculate_control_outputs()
        
        # 3. Create and publish the control message
        control_msg = Control()
        control_msg.steering = self.steering_output
        control_msg.throttle = float(self.throttle_output)
        control_msg.brake = float(self.brake_output) # Will always be 0.0
        control_msg.latswitch = 0  # Lateral control is off
        control_msg.longswitch = 1 # We are controlling longitudinally
        
        self.control_pub.publish(control_msg)
        
        # Periodic logging for monitoring
        if not hasattr(self, 'last_log_time'): self.last_log_time = 0
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_log_time > 1.0:  # Log every second
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
        if test_node:
            test_node.get_logger().info("Staged Acceleration Test shutting down...")
    except Exception as e:
        if test_node:
            test_node.get_logger().error(f"Unhandled exception: {e}")
        else:
            print(f"Exception before node initialization: {e}")
    finally:
        if test_node:
            # Ensure a final stop command is sent on shutdown
            try:
                final_control = Control(throttle=0.0, brake=0.0, steering=0.0, longswitch=1, latswitch=0)
                test_node.control_pub.publish(final_control)
                test_node.get_logger().info("Sent final zero-throttle command.")
                time.sleep(0.1)
            except Exception as e:
                print(f"Error sending final command: {e}")
            test_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
