#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
# from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from sensor_msgs.msg import BatteryState

class bluerov2_sensors(Node):
    def __init__(self):
        super().__init("bluerov2_sensors")
        self.subscriber = self.create_subscription(
            BatteryState,
            "/mavros/battery",
            self.callback,
            10
        )
        self.subscriber = self.create_subscription(
            Imu,
            "/mavros/imu/data",
            self.callback,
            10
        )
        self.subscriber
        self.get_logger().info("starting subscriber node")
        self.declare_parameter('Battery')
        self.declare_parameter('Imu')

    def callback(self, msg):
        pass