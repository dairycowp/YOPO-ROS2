#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import tf_transformations as tfs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3

class OdomSender(Node):
    def __init__(self):
        super().__init__('odom_sender')
        
        self.pub = self.create_publisher(Odometry, 'odom', 10)
        
        # Create timer for publishing at 1 Hz
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0
        
        # Create the message
        self.msg = Odometry()
        self.msg.header.frame_id = "world"

        q = tfs.quaternion_from_euler(0, 0, 0, "rzyx")

        self.msg.pose.pose.position = Point(x=0.0, y=0.0, z=0.0)
        self.msg.twist.twist.linear = Vector3(x=0.0, y=0.0, z=0.0)
        self.msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.get_logger().info(str(self.msg))

    def timer_callback(self):
        self.counter += 1
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)
        self.get_logger().info("Send %3d msg(s)." % self.counter)


def main(args=None):
    rclpy.init(args=args)
    node = OdomSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
