#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from mavros_msgs.srv import CommandBool
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

class Arm(Node):
    '''
    Node to arm and disarm ROV'''
    def __init__(self):
        '''
        Initializes node, cals inherited Node class
        Creates client, then waits for service request
        '''
        super().__init__("ARM")
        self.cli = self.create_client(CommandBool,'/mavros/cmd/arming')
        while not self.cli.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = CommandBool.Request()

    def arm_request(self, value):
        '''
        Arming Request, assigns value to request
        asynchronously calls
        '''
        self.req.value = value
        self.future = self.cli.call_async(self.req)
        return self.future

def main(args = None):
    rclpy.init(args=args)
    node = Arm()
    arm(node,True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        arm(node,False)
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

def arm(node,value):
    '''
    refactor of code, takes in value and requests service
    
    value: boolean in true or false
    '''
    future = node.arm_request(value)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    if (value): 
        node.get_logger().info("Robot has been ARMED")
    if (not value): 
        node.get_logger().info("Robot has been DISARMED")

if __name__=="__main__":
    main()