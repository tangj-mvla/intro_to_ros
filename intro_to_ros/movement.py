#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node

from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

class Movement(Node):
    def __init__(self):
        super.__init__("movement")
        