#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

class Movement(Node):
    def __init__(self):
        super().__init__("movement")
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.SYSTEM_DEFAULT,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.publisher = self.create_publisher(
            OverrideRCIn,
            "/mavros/rc/override",
            qos_profile
        )
        self.get_logger().info("starting publish")
        # self.publisher_timer = self.create_timer(
        #         1.0, self.roll(2000)
        # )
        
        self.get_logger().info("SUBSCRIBERS: " + str(self.publisher.get_subscription_count()))
                               
    def roll(self, pwm, t):
        msg = OverrideRCIn()
        msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * 18
        msg.channels[1] = pwm
        self.publisher.publish(msg)
        time.sleep(t)

    def vertical(self, pwm, t):
        msg = OverrideRCIn()
        msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * 18
        msg.channels[2] = pwm
        self.publisher.publish(msg)
        time.sleep(t)

    def yaw(self, pwm, t):
        msg = OverrideRCIn()
        msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * 18
        msg.channels[3] = pwm
        self.publisher.publish(msg)
        time.sleep(t)

    def forward(self, pwm, t):
        msg = OverrideRCIn()
        msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * 18
        msg.channels[4] = pwm
        self.publisher.publish(msg)
        time.sleep(t)

    def strafe(self, pwm, t):
        msg = OverrideRCIn()
        msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * 18
        msg.channels[5] = pwm
        self.publisher.publish(msg)
        time.sleep(t)

    def neutral(self):
        msg = OverrideRCIn()
        msg.channels = [1500] * 18
        self.publisher.publish(msg)

    def override_node(self):
        msg = OverrideRCIn()
        msg.channels = [1900,1900,1900,
                        1900,1900,1900,
                        OverrideRCIn.CHAN_NOCHANGE,OverrideRCIn.CHAN_NOCHANGE,OverrideRCIn.CHAN_NOCHANGE,
                        OverrideRCIn.CHAN_NOCHANGE,OverrideRCIn.CHAN_NOCHANGE,OverrideRCIn.CHAN_NOCHANGE,
                        OverrideRCIn.CHAN_NOCHANGE,OverrideRCIn.CHAN_NOCHANGE,OverrideRCIn.CHAN_NOCHANGE,
                        OverrideRCIn.CHAN_NOCHANGE,OverrideRCIn.CHAN_NOCHANGE,OverrideRCIn.CHAN_NOCHANGE]
        # msg.channels = [1000,1100,1200,1300,1400,1500,1600,1700]
        # msg.rssi = 0 
        self.publisher.publish(msg)
        # channels go 1,2,3,4,5,...
        # index 0 is channel 1, index 1 is channel 2, etc.
        # 
        # roll: channel 2
        # up/down: channel 3
        # rotate: channel 4
        # forward: channel 5
        # strafe: channel 6

def hot(node):
    node.rotate(1300,0.6)

def cross(node):   
    node.rotate(1700,0.6)

def buns(node):
    node.rotate(1300,1.2)


def hotCrossBuns(node):
    node.
    pass

def pennies(node):
    
    pass

def children(node):
    node.roll(1700,0.6)
    pass

def rave(node):
    
    pass

def main(args=None):
    rclpy.init(args=args)

    node = Movement()
    # beginning
    node.strafe(1700,1)
    node.strafe(1300,1)
    node.strafe(1700,1)
    node.strafe(1300,1)
    node.strafe(1700,1)
    node.strafe(1300,1)
    node.strafe(1700,1)
    node.strafe(1300,1)
    node.strafe(1700,1)
    node.strafe(1300,0.3)
    
    # hot cross buns funciton

    node.neutral()
    
    #  node.strafe(1700)
    # node.neutral()
    # node.forward(2000)
    # time.sleep(5)
    # node.destroy_node()
    # if rclpy.ok():
    #     rclpy.shutdown()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        print ("shutting down")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=="__main__":
    main()




