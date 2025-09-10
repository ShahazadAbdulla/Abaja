#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import time
from enum import Enum

from vehiclecontrol.msg import Control

class AEBSTATE(Enum)
    ACCELERATING = "accelerating"
    BRAKING = "braking"

class Obstacletype(Enum)
    UNKNOWN = "unknown"
    CAR = "car_or_similar_width vehicles"
    PEDESTRIAN = "pedestrian"
    MOTORCYCLE = "motorcycle_cycle"

class AEBcontrollernode()
    def __init__ (self)
        super().__init__('aeb_controller_node')
        self.get_logger().info("Starting Permanent AEB Controller...")

 
        self.TARGET_SPEED_MS = self.TARGET_SPEED_KMPH / 3.6  # Convert to m/s
        self.ACCELERATION_DISTANCE = 40.0  # Accelerate over 100 meters
        self.BRAKING_DISTANCE_THRESHOLD = 6.0  # Emergency stop at 6 meters
        self.BRAKING_DECELERATION = 8.0  # Brake at 8 m/s²
        self.RADAR_MONITORING_RANGE = 40.0  # Monitor obstacles from 40 meters

