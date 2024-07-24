#!/usr/bin/env python3
import numpy as np 
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import Altitude, ManualControl

class depthControl(Node):
    '''
    node to control depth based on PID control calculations

    integral + proportional + derivative
    '''
    error_accumulator = 0
    previous_error = 0
    def __init__(self):
        super().__init("depthControl")
        self.measuredDepth = self.create_subscription(
            Altitude,
            "bluerov2/depth",
            self.measuredDepthCallback,
            10
        )
        self.get_logger().info("Starting measured depth subscription")
        self.desiredDepth = self.create_subscription(
            Altitude,
            "bluerov2/desired_depth",
            self.desiredDepthCallback,
            10
        )
        self.get_logger().info("Starting desired depth subscription")
        self.publisher = self.create_publisher(
            ManualControl, 
            "bluerov2/manual_control",
            10
        )
        self.get_logger().info("Starting publsher")
        self.measured_depth = Altitude()
        self.desired_depth = Altitude()

    def measuredDepthCallback(self, msg):
        self.measured_depth = msg
        self.depth_control(1)
        self.get_logger().info(f"\nMeasured Depth: {msg.local}")
        
    def desiredDepthCallback(self,msg):
        self.desired_depth = msg
        self.depth_control(1)
        self.get_logger().info(f"\nDesired Depth: {msg.local}")
    
    def depth_control(self, dt):
        # initial part, getting value, ect
        measured_position = self.measured_depth.local
        desired_position = self.desired_depth.local
        
        # proportional control
        Kp = 1.0
        error = desired_position - measured_position
        self.proportional = Kp * error

        # integral control
        Ki = 1.0
        # self.integral = self.desired_depth - measured_position
        self.error_accumulator += error * dt # DT = TIME SINCE LAST UPDATE
        
        self.integral = min(Ki * self.error_accumulator, 1.0) # PREVENTS INTEGRAL WINDUP PAST 1.0

        # derivative control
        Kd = 1.0
        self.derivative = Kd * (error - self.previous_error) / dt 
        self.previous_error = error

        # add all & publish        
        self.pid = self.proportional + self.integral + self.derivative
        self.getlogger().info(f"PID: {self.pid}")
        msg = ManualControl()
        
        msg.z = self.pid
        self.publisher.publish(msg)
        
def main(args = None):
    rclpy.init(args = args)
    node = depthControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=="__main__":
    main()