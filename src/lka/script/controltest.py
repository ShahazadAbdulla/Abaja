#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Range
import time
import math

class AEBController(Node):
    def __init__(self):
        super().__init__('aeb_controller')
        
        # Constants for AEB
        self.MAX_SPEED = 10.0  # m/s
        self.SAFE_DISTANCE = 10.0  # meters
        self.BRAKE_THRESHOLD = 5.0  # meters
        self.EMERGENCY_BRAKE_THRESHOLD = 3.0  # meters
        self.TARGET_DISTANCE = 40.0  # meters
        self.MAX_DECEL = 8.0  # m/s²
        self.MIN_DECEL = 5.0  # m/s²
        
        # Current state
        self.current_speed = 0.0
        self.distance_to_obstacle = float('inf')
        self.distance_traveled = 0.0
        self.last_time = time.time()
        
        # Publishers
        self.control_pub = self.create_publisher(
            Vector3,
            '/vehicleControl',
            10
        )
        
        # Subscribers
        self.range_sub = self.create_subscription(
            Range,
            '/front_distance',
            self.range_callback,
            10
        )
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10Hz
        
        self.get_logger().info('AEB Controller initialized')

    def range_callback(self, msg):
        self.distance_to_obstacle = msg.range

    def update_distance_traveled(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.distance_traveled += abs(self.current_speed * dt)
        self.last_time = current_time

    def calculate_deceleration(self):
        # Calculate required deceleration to stop at 40m
        remaining_distance = self.TARGET_DISTANCE - self.distance_traveled
        if remaining_distance <= 0:
            return self.MAX_DECEL
        
        # Using v² = u² + 2as, where v=0 (final stop)
        decel = (self.current_speed ** 2) / (2 * remaining_distance)
        # Clamp deceleration between MIN_DECEL and MAX_DECEL
        return min(max(decel, self.MIN_DECEL), self.MAX_DECEL)

    def control_loop(self):
        self.update_distance_traveled()
        
        # Initialize control message
        control_msg = Vector3(x=0.0, y=0.0, z=0.0)
        
        # Check if target distance reached
        if self.distance_traveled >= self.TARGET_DISTANCE:
            control_msg.y = 0.0  # Stop forward motion
            control_msg.z = 0.0  # No brake
            self.current_speed = 0.0
            self.get_logger().info('Target distance reached. Vehicle stopped.')
            self.control_pub.publish(control_msg)
            return

        # Calculate control commands based on AEB logic
        if self.distance_to_obstacle <= self.EMERGENCY_BRAKE_THRESHOLD:
            # Emergency braking
            control_msg.y = 0.0
            control_msg.z = self.MAX_DECEL
            self.current_speed = 0.0
            self.get_logger().warn('Emergency braking activated!')
            
        elif self.distance_to_obstacle <= self.BRAKE_THRESHOLD:
            # Gradual braking
            brake_intensity = (self.BRAKE_THRESHOLD - self.distance_to_obstacle) / self.BRAKE_THRESHOLD
            decel = brake_intensity * self.MAX_DECEL
            control_msg.y = max(0.0, self.current_speed - decel * 0.1)
            control_msg.z = decel
            self.current_speed = control_msg.y
            self.get_logger().info(f'Gradual braking: {decel:.2f} m/s²')
            
        elif self.distance_traveled > (self.TARGET_DISTANCE * 0.8):  # Start slowing at 80% of target distance
            # Calculate deceleration needed to stop at target
            decel = self.calculate_deceleration()
            control_msg.y = max(0.0, self.current_speed - decel * 0.1)
            control_msg.z = decel
            self.current_speed = control_msg.y
            self.get_logger().info(f'Approaching target: Decel={decel:.2f} m/s²')
            
        else:
            # Normal driving
            control_msg.y = self.MAX_SPEED
            control_msg.z = 0.0
            self.current_speed = self.MAX_SPEED
            
        # Publish control command
        self.control_pub.publish(control_msg)
        self.get_logger().info(f'Distance: {self.distance_traveled:.2f}m, Speed: {self.current_speed:.2f}m/s')

def main(args=None):
    rclpy.init(args=args)
    aeb_controller = AEBController()

    try:
        rclpy.spin(aeb_controller)
    except KeyboardInterrupt:
        aeb_controller.get_logger().info('Keyboard interrupt received. Shutting down...')
    except Exception as e:
        aeb_controller.get_logger().error(f'Error occurred: {str(e)}')
    finally:
        # Send final stop command
        stop_msg = Vector3(x=0.0, y=0.0, z=0.0)
        aeb_controller.control_pub.publish(stop_msg)
        aeb_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()