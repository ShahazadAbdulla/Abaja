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

