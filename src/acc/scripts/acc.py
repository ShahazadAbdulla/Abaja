#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from enum import Enum

# Import required message types
from radar_msgs.msg import RadarTrack
from feedback.msg import Velocity
from vehiclecontrol.msg import Control

# Simplified state machine based on the new logic
class SimpleACCState(Enum):
    CRUISING = "cruising_to_target_speed"
    FOLLOWING = "following_lead_vehicle"

class SimpleACCController(Node):
    def __init__(self):
        super().__init__('simple_acc_controller')
        self.get_logger().info("Starting Simplified ACC Controller Node...")
        
        # === Core ACC Parameters ===
        self.TARGET_SPEED_KMPH = 30.0
        self.TARGET_SPEED_MS = self.TARGET_SPEED_KMPH / 3.6

        # Distance to start tracking a lead vehicle
        self.TRACKING_ENGAGE_DISTANCE = 30.0 
        
        # The ideal distance to maintain behind a lead vehicle
        self.TARGET_FOLLOW_DISTANCE = 4.0 

        # === Control Gains (How aggressively the car reacts) ===
        # Gain for cruising speed control
        self.KP_SPEED = 0.4 
        # Gain for following distance control
        self.KP_GAP = 0.15

        # === State Variables ===
        self.current_speed_ms = 0.0
        self.leading_distance = float('inf')
        self.acc_state = SimpleACCState.CRUISING

        # === ROS2 Subscribers and Publishers ===
        self.radar_sub = self.create_subscription(
            RadarTrack, 'RadarObjects', self.radar_callback, 10)
        self.velocity_sub = self.create_subscription(
            Velocity, 'VehicleSpeed', self.velocity_callback, 10)
        self.control_pub = self.create_publisher(Control, '/vehicle_control', 10)
        
        # Main control loop timer (runs 50 times per second)
        self.timer = self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info("Simplified ACC Controller initialized.")

    def radar_callback(self, msg: RadarTrack):
        """
        Processes radar data to find the closest vehicle directly in front.
        """
        closest_distance_in_path = float('inf')
        
        for obj in msg.objects:
            # Consider objects that are in front and within a 1.5-meter lateral path
            # --- THIS IS THE MODIFIED LINE ---
            if obj.x_distance > 0.1 and abs(obj.y_distance) <= 1.5: 
                if obj.x_distance < closest_distance_in_path:
                    closest_distance_in_path = obj.x_distance
        
        self.leading_distance = closest_distance_in_path
            
    def velocity_callback(self, msg: Velocity):
        """
        Updates the vehicle's current speed from feedback.
        """
        self.current_speed_ms = msg.vehicle_velocity
        
    def update_acc_state(self):
        """
        Switches between CRUISING and FOLLOWING based on the tracking distance.
        """
        previous_state = self.acc_state
        
        if self.leading_distance < self.TRACKING_ENGAGE_DISTANCE:
            self.acc_state = SimpleACCState.FOLLOWING
        else:
            self.acc_state = SimpleACCState.CRUISING
        
        if self.acc_state != previous_state:
            self.get_logger().info(f"ACC State Change: {previous_state.name} -> {self.acc_state.name}")

    def calculate_cruising_throttle(self):
        """
        Calculates throttle to reach the target speed when no car is in front.
        Returns only throttle, as braking is not needed for simple cruising.
        """
        speed_error = self.TARGET_SPEED_MS - self.current_speed_ms
        
        # Calculate throttle using a simple proportional controller
        throttle = self.KP_SPEED * speed_error
        
        # Clamp the throttle value to be between 0.0 and a safe max of 0.7
        throttle = max(0.0, min(throttle, 0.7))
        
        return throttle, 0.0 # Returns (throttle, brake)

    def calculate_following_control(self):
        """
        Calculates throttle and brake to maintain the target follow distance.
        """
        throttle, brake = 0.0, 0.0
        
        # Calculate the error: positive if we are too far, negative if too close
        gap_error = self.leading_distance - self.TARGET_FOLLOW_DISTANCE
        
        # Calculate the control adjustment based on the error
        control_adjustment = self.KP_GAP * gap_error
        
        if control_adjustment > 0:
            # If we are too far away (error is positive), apply throttle
            throttle = control_adjustment
        else:
            # If we are too close (error is negative), apply brake
            brake = abs(control_adjustment)
            
        # Clamp the outputs to be within the valid range [0.0, 1.0]
        throttle = max(0.0, min(throttle, 1.0))
        brake = max(0.0, min(brake, 1.0))
        
        return throttle, brake

    def control_loop(self):
        """
        The main loop that runs continuously to control the vehicle.
        """
        # 1. Determine if we should be cruising or following
        self.update_acc_state()
        
        throttle, brake = 0.0, 0.0
        
        # 2. Calculate throttle and brake based on the current state
        if self.acc_state == SimpleACCState.CRUISING:
            throttle, brake = self.calculate_cruising_throttle()
        elif self.acc_state == SimpleACCState.FOLLOWING:
            throttle, brake = self.calculate_following_control()
        
        # 3. Create and publish the final control message
        control_msg = Control()
        control_msg.throttle = throttle
        control_msg.brake = brake
        control_msg.steering = 0.0 
        control_msg.longswitch = 1 # Enable longitudinal (throttle/brake) control
        control_msg.latswitch = 0   
        self.control_pub.publish(control_msg)
        
        # Log a summary of the status periodically
        if not hasattr(self, '_summary_log_counter'): self._summary_log_counter = 0
        self._summary_log_counter = (self._summary_log_counter + 1)
        if self._summary_log_counter % 50 == 0: # Log every 1 second
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
            # Send a final safe command on shutdown
            final_control = Control(throttle=0.0, brake=1.0, longswitch=1)
            acc_controller_node.control_pub.publish(final_control)
            acc_controller_node.get_logger().info("Sent final safe brake command.")
            time.sleep(0.1)
            acc_controller_node.destroy_node() 
        if rclpy.ok(): 
            rclpy.shutdown() 

if __name__ == '__main__':
    main()