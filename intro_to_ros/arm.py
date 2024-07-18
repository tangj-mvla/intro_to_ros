#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from mavros_msgs.srv import CommandBool
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

class Arm(Node):
    def __init__(self):
        super().__init__("ARM")
        self.cli = self.create_client(CommandBool,'/mavros/cmd/arming')
        while not self.cli.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = CommandBool.Request()

    def arm_request(self, value):
        self.req.value = value
        self.future = self.cli.call_async(self.req)
        return self.future

def main(args = None):
    rclpy.init(args=args)
    node = Arm()
    
    try:
        arm(node,True)
        rclpy.spin(node)
    except KeyboardInterrupt:
        arm(node, False)
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        # future = node.arm_request(False)
        # rclpy.spin_until_future_complete(node = node, future = future)
        # response = future.result()
        # node.get_logger().info('Robot has been DISARMED')
        # print("\nKeyboardInterrupt received, shutting down...")
        # arm(node,False)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

def arm(node,value):
    future = node.arm_request(value)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    # node.get_logger().info(str(response.success))
    if (value): 
        node.get_logger().info("Robot has been ARMED")
    if (not value): 
        node.get_logger().info("Robot has been DISARMED")

if __name__=="__main__":
    main()