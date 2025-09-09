#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from enum import Enum

from vehiclecontrol.msg import Control
from radar_msgs.msg import RadarTrack
from feedback.msg import Velocity

class TestState(Enum):
    """
    Simplified state machine for the acceleration test.
    """
    RAMPING = "ramping_throttle"
    HOLDING_MAX = "holding_max_throttle"
    MOTOR_OFF = "motor_off"
    OBSTACLE_BRAKE = "obstacle_detected_braking"

class AccelerationTestNode(Node):
    """
    ROS 2 node for a staged acceleration test with safety braking.
    """
    def __init__(self):
        super().__init__('acceleration_test_node')
        self.get_logger().info("Starting Staged Acceleration Test Node...")

        # === TEST PARAMETERS ===
        self.THROTTLE_START_VALUE = 0.1
        self.THROTTLE_END_VALUE = 1.0
        self.THROTTLE_STEP = 0.1
        self.RAMP_INTERVAL_SECONDS = 2.0  # Time to hold each throttle step
        self.MAX_HOLD_DURATION = 4.0  # Time to hold max throttle before turning off motor
        self.MAX_SPEED_KMPH = 26.0
        # Convert max speed from km/h to m/s
        self.MAX_SPEED_MPS = self.MAX_SPEED_KMPH * 1000.0 / 3600.0
        self.OBSTACLE_BRAKE_DISTANCE = 6.0
        self.target_speed_for_current_throttle = 0.0

        # === State variables ===
        self.current_speed_mps = 0.0
        self.current_state = TestState.RAMPING
        self.current_throttle_level = 0.0
        self.last_ramp_time = self.get_clock().now()
        self.max_throttle_start_time = None
        self.obstacle_detected = False

        # Control outputs
        self.throttle_output = 0.0
        self.brake_output = 0.0
        self.steering_output = 0.0

        # Subscriptions
        self.vel_sub = self.create_subscription(
            Velocity,
            '/vehicle_velocity',
            self.velocity_callback,
            10
        )
        self.radar_sub = self.create_subscription(
            RadarTrack,
            '/radar/track_list',
            self.radar_callback,
            10
        )

        # Control publisher
        self.control_pub = self.create_publisher(Control, '/vehicle_control', 10)
        
        # Control loop timer (50Hz = 0.02s)
        self.control_timer = self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info("Staged Acceleration Test Node initialized successfully.")

    def velocity_callback(self, msg: Velocity):
        """
        Callback to receive vehicle velocity data.
        The Velocity message is assumed to contain a float32 `vehicle_velocity` field.
        """
        self.current_speed_mps = msg.vehicle_velocity

    def radar_callback(self, msg: RadarTrack):
        """
        Callback to receive radar data and check for obstacles.
        """
        self.obstacle_detected = False
        for track in msg.tracks:
            # Check the distance of each tracked object.
            # Assuming x_distance is the forward distance.
            if track.x_distance <= self.OBSTACLE_BRAKE_DISTANCE:
                self.obstacle_detected = True
                self.get_logger().warn(f"Obstacle detected within {self.OBSTACLE_BRAKE_DISTANCE}m at x={track.x_distance:.2f}m. Initiating brake.")
                return # Stop checking once one obstacle is found.

    def determine_vehicle_state(self):
        """
        Determine the current state based on test progress, speed limits, and obstacle detection.
        """
        # Highest priority: Check for an obstacle
        if self.obstacle_detected:
            return TestState.OBSTACLE_BRAKE

        # Check for maximum speed limit
        if self.current_speed_mps >= self.MAX_SPEED_MPS:
            if self.current_state != TestState.MOTOR_OFF:
                self.get_logger().info(f"Maximum speed of {self.MAX_SPEED_KMPH} km/h reached. Shutting down test.")
            return TestState.MOTOR_OFF

        # Standard state transitions for the test sequence
        if self.current_state == TestState.MOTOR_OFF:
            return TestState.MOTOR_OFF
        
        if self.current_throttle_level >= self.THROTTLE_END_VALUE:
            if self.current_state != TestState.HOLDING_MAX:
                self.max_throttle_start_time = self.get_clock().now()
                self.get_logger().info("Reached maximum throttle. Starting countdown to motor off.")
                return TestState.HOLDING_MAX
            
            if self.max_throttle_start_time is not None:
                now = self.get_clock().now()
                time_at_max = (now - self.max_throttle_start_time).nanoseconds / 1e9
                if time_at_max >= self.MAX_HOLD_DURATION:
                    self.get_logger().info("Countdown complete. Turning motor off (throttle = 0.0).")
                    return TestState.MOTOR_OFF
            
            return TestState.HOLDING_MAX

        return TestState.RAMPING

    def calculate_control_outputs(self):
        """
        Calculate throttle based on the current state of the test.
        """
        # Brake is applied by setting throttle to zero for simplicity in this logic
        self.brake_output = 0.0
        
        if self.current_state == TestState.OBSTACLE_BRAKE:
            self.throttle_output = 0.0
            
        elif self.current_state == TestState.RAMPING:
            if self.current_throttle_level < self.THROTTLE_START_VALUE:
                self.current_throttle_level = self.THROTTLE_START_VALUE
                self.get_logger().info(f"Starting test. Setting initial throttle to {self.current_throttle_level:.1f}")

            # Check if it's time to increase the throttle based on interval
            now = self.get_clock().now()
            time_since_last_ramp = (now - self.last_ramp_time).nanoseconds / 1e9
            
            if time_since_last_ramp >= self.RAMP_INTERVAL_SECONDS:
                # Ramp up
                self.current_throttle_level += self.THROTTLE_STEP
                self.current_throttle_level = min(self.current_throttle_level, self.THROTTLE_END_VALUE)
                self.get_logger().info(f"Ramping up: Setting throttle to {self.current_throttle_level:.1f}")
                self.last_ramp_time = now # Reset the timer for the next interval
            
            self.throttle_output = self.current_throttle_level

        elif self.current_state == TestState.HOLDING_MAX:
            self.throttle_output = self.THROTTLE_END_VALUE
            
        elif self.current_state == TestState.MOTOR_OFF:
            self.throttle_output = 0.0

    def control_loop(self):
        """
        Main control loop executed at 50Hz.
        """
        self.current_state = self.determine_vehicle_state()
        self.calculate_control_outputs()
        
        control_msg = Control()
        control_msg.steering = self.steering_output
        control_msg.throttle = float(self.throttle_output)
        control_msg.brake = float(self.brake_output)
        control_msg.latswitch = 0
        control_msg.longswitch = 1
        
        self.control_pub.publish(control_msg)
        
        if not hasattr(self, 'last_log_time'): self.last_log_time = self.get_clock().now()
        current_time = self.get_clock().now()
        if (current_time - self.last_log_time).nanoseconds / 1e9 > 1.0:
            self.last_log_time = current_time
            self.get_logger().info(
                f"State: {self.current_state.value} | "
                f"Throttle Cmd: {self.throttle_output:.2f} | "
                f"Speed: {self.current_speed_mps*3.6:.1f} km/h"
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
