#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

class Movement(Node):
    def __init__(self):
        super.__init__("movement")
        qosProfile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.create_publisher(
            Vector3Stamped,
            "/mavros/manual_control/send",
            qosProfile
        )
