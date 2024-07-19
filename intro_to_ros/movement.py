#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn, RCIn, RCOut, ManualControl
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from pymavlink import mavutil

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
        self.publisher_timer = self.create_timer(
            5.0, self.override_node
        )
        self.get_logger().info("SUBSCRIBERS: " + str(self.publisher.get_subscription_count()))

    
    def override_node(self):
        msg = OverrideRCIn()
        
        msg.channels = [OverrideRCIn.CHAN_NOCHANGE,OverrideRCIn.CHAN_NOCHANGE,OverrideRCIn.CHAN_NOCHANGE,
                        OverrideRCIn.CHAN_NOCHANGE,1700,OverrideRCIn.CHAN_NOCHANGE,
                        OverrideRCIn.CHAN_NOCHANGE,OverrideRCIn.CHAN_NOCHANGE,OverrideRCIn.CHAN_NOCHANGE,
                        OverrideRCIn.CHAN_NOCHANGE,OverrideRCIn.CHAN_NOCHANGE,OverrideRCIn.CHAN_NOCHANGE,
                        OverrideRCIn.CHAN_NOCHANGE,OverrideRCIn.CHAN_NOCHANGE,OverrideRCIn.CHAN_NOCHANGE,
                        OverrideRCIn.CHAN_NOCHANGE,OverrideRCIn.CHAN_NOCHANGE,OverrideRCIn.CHAN_NOCHANGE]
        # msg.channels = [1000,1100,1200,1300,1400,1500,1600,1700]
        # msg.rssi = 0 
        self.publisher.publish(msg)
        mav1 = mavutil.mavlink_connection()
        mavutil.mavlink_connection(m)
        self.get_logger().info(f"Inputs: {msg.channels[0]}, {msg.channels[1]}")

        # forward: channel 5
        # rotate: channel 4
        # strafe: channel 6
        # up/down: channel 3

def main(args=None):
    rclpy.init(args=args)
    node = Movement()

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




