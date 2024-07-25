#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import Altitude, ManualControl

class depthControl(Node):
    '''
    node to control depth based on PID control calculations

    PID = integral + proportional + derivative
    '''
    error_accumulator = 0
    previous_error = 0
    t1 = 0
    t2 = 0
    max_integral = 6.0
    max_throttle = 100.0
    desired_position = None
    
    def __init__(self):

        super().__init__("depthControl")
        
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
        self.previous_error = 0.0
        
        # self.timer = self.create_timer(0.01, self.depth_control)

    def measuredDepthCallback(self, msg):
        self.measured_depth = msg
        if (self.t1 == 0):
            self.t1 = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            self.t2 = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        else:
            self.t1 = self.t2
            self.t2 = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        self.depth_control()
        self.get_logger().info(f"\nMeasured Depth: {msg.local}")
        
    def desiredDepthCallback(self, msg):
        self.desired_position = msg.local
        self.get_logger().info(f"\nDesired Depth: {msg.local}")
    
    def depth_control(self):
        msg = ManualControl()
        # initial part, getting value, ect
        measured_position = self.measured_depth.local #og local
        # self.desired_position = self.desired_depth
        self.get_logger().info(f"\nDesired Depth: {self.desired_position}")
        if (self.desired_position is None): return
        # desired_position = 0.5
        dt = self.t2 - self.t1
        self.get_logger().info(f"\n\nt1: {self.t1}")
        self.get_logger().info(f"\nt2: {self.t2}")
        self.get_logger().info(f"\ndt: {dt}")
        # constants
        Kp = 150.0
        Ki = 250.0
        Kd = 300.0
        # proportional control
        error = self.desired_position - measured_position
        self.get_logger().info(f"\nError: {error}")
        self.proportional = Kp * error
        self.get_logger().info(f"\nProportional: {self.proportional}")

        # integral control
        self.error_accumulator += error * dt # DT = TIME SINCE LAST UPDATE
        self.get_logger().info(f"\nError Accumulator: {self.error_accumulator}")
        self.integral = min(max(Ki * self.error_accumulator,-self.max_integral), self.max_integral) # PREVENTS INTEGRAL WINDUP PAST 1.0 & -1.0
        self.get_logger().info(f"\nIntegral: {self.integral}")

        # derivative control
        if (dt == 0): 
            self.derivative = 0
        else:
            self.derivative = Kd * (error - self.previous_error) / dt 
        self.previous_error = error
        self.get_logger().info(f"\nPrevious Error: {self.previous_error}")
        self.get_logger().info(f"\nDerivative: {self.derivative}")

        # add all & publish 
        msg.z = float((self.proportional + self.integral + self.derivative) * -1)
        msg.z = min(max(msg.z, -self.max_throttle), self.max_throttle)

        self.get_logger().info(f"\nPID: {msg.z}")
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