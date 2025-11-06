#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.timer import Timer
import numpy as np
import tf2_ros
import tf_transformations as tfs
from math import pi
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy
from builtin_interfaces.msg import Time as TimeMsg


class OdometryConverter:
    def __init__(self, node, frame_id_in_, frame_id_out_, broadcast_tf_, body_frame_id_, intermediate_frame_id_, world_frame_id_):
        self.node = node
        self.frame_id_in = frame_id_in_
        self.frame_id_out = frame_id_out_
        self.broadcast_tf = broadcast_tf_
        self.body_frame_id = body_frame_id_
        self.intermediate_frame_id = intermediate_frame_id_
        self.world_frame_id = world_frame_id_
        self.in_odom_sub = None
        self.out_odom_pub = None
        self.out_path_pub = None
        self.path_pub_timer = None
        self.tf_pub_flag = True
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(node)
        
        if self.broadcast_tf:
            self.node.get_logger().info('ROSTopic: [%s]->[%s] TF: [%s]-[%s]-[%s]' %
                          (self.frame_id_in, self.frame_id_out, self.body_frame_id, self.intermediate_frame_id, self.world_frame_id))
        else:
            self.node.get_logger().info('ROSTopic: [%s]->[%s] No TF' %
                          (self.frame_id_in, self.frame_id_out))

        self.path = []

    def in_odom_callback(self, in_odom_msg):
        q = np.array([in_odom_msg.pose.pose.orientation.x,
                      in_odom_msg.pose.pose.orientation.y,
                      in_odom_msg.pose.pose.orientation.z,
                      in_odom_msg.pose.pose.orientation.w])
        p = np.array([in_odom_msg.pose.pose.position.x,
                      in_odom_msg.pose.pose.position.y,
                      in_odom_msg.pose.pose.position.z])

        e = tfs.euler_from_quaternion(q, 'rzyx')
        wqb = tfs.quaternion_from_euler(e[0], e[1], e[2], 'rzyx')
        wqc = tfs.quaternion_from_euler(e[0],  0.0,  0.0, 'rzyx')

        #### odom ####
        odom_msg = in_odom_msg
        assert(in_odom_msg.header.frame_id == self.frame_id_in)
        odom_msg.header.frame_id = self.frame_id_out
        odom_msg.child_frame_id = ""
        self.out_odom_pub.publish(odom_msg)

        #### tf ####
        if self.broadcast_tf and self.tf_pub_flag:
            self.tf_pub_flag = False
            
            # Send transform from frame_id_in to frame_id_out
            if not self.frame_id_in == self.frame_id_out:
                t = TransformStamped()
                t.header.stamp = odom_msg.header.stamp
                t.header.frame_id = self.frame_id_out
                t.child_frame_id = self.frame_id_in
                t.transform.translation.x = 0.0
                t.transform.translation.y = 0.0
                t.transform.translation.z = 0.0
                q_tf = tfs.quaternion_from_euler(0.0, 0.0, 0.0, 'rzyx')
                t.transform.rotation.x = q_tf[0]
                t.transform.rotation.y = q_tf[1]
                t.transform.rotation.z = q_tf[2]
                t.transform.rotation.w = q_tf[3]
                self.tf_broadcaster.sendTransform(t)

            # Send transform from world_frame_id to frame_id_out
            if not self.world_frame_id == self.frame_id_out:
                t = TransformStamped()
                t.header.stamp = odom_msg.header.stamp
                t.header.frame_id = self.frame_id_out
                t.child_frame_id = self.world_frame_id
                t.transform.translation.x = 0.0
                t.transform.translation.y = 0.0
                t.transform.translation.z = 0.0
                q_tf = tfs.quaternion_from_euler(0.0, 0.0, 0.0, 'rzyx')
                t.transform.rotation.x = q_tf[0]
                t.transform.rotation.y = q_tf[1]
                t.transform.rotation.z = q_tf[2]
                t.transform.rotation.w = q_tf[3]
                self.tf_broadcaster.sendTransform(t)

            # Send transform from world_frame_id to body_frame_id
            t = TransformStamped()
            t.header.stamp = odom_msg.header.stamp
            t.header.frame_id = self.world_frame_id
            t.child_frame_id = self.body_frame_id
            t.transform.translation.x = p[0]
            t.transform.translation.y = p[1]
            t.transform.translation.z = p[2]
            t.transform.rotation.x = wqb[0]
            t.transform.rotation.y = wqb[1]
            t.transform.rotation.z = wqb[2]
            t.transform.rotation.w = wqb[3]
            self.tf_broadcaster.sendTransform(t)

            # Send transform from world_frame_id to intermediate_frame_id
            t = TransformStamped()
            t.header.stamp = odom_msg.header.stamp
            t.header.frame_id = self.world_frame_id
            t.child_frame_id = self.intermediate_frame_id
            t.transform.translation.x = p[0]
            t.transform.translation.y = p[1]
            t.transform.translation.z = p[2]
            t.transform.rotation.x = wqc[0]
            t.transform.rotation.y = wqc[1]
            t.transform.rotation.z = wqc[2]
            t.transform.rotation.w = wqc[3]
            self.tf_broadcaster.sendTransform(t)
            
        #### path ####
        pose = PoseStamped()
        pose.header = odom_msg.header
        pose.pose.position.x = p[0]
        pose.pose.position.y = p[1]
        pose.pose.position.z = p[2]
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.path.append(pose)

    def path_pub_callback(self):
        if self.path:
            path = Path()
            path.header = self.path[-1].header
            path.poses = self.path[-30000::1]
            self.out_path_pub.publish(path)

    def tf_pub_callback(self):
        self.tf_pub_flag = True


class TfAssist(Node):
    def __init__(self):
        super().__init__('tf_assist')

        self.converters = []
        self.timers = []  # Keep references to timers to prevent garbage collection
        
        # For now, we'll use default values
        # In a full implementation, these would be read from parameters
        frame_id_in = 'odom'
        frame_id_out = 'world'
        broadcast_tf = True
        body_frame_id = 'body'
        intermediate_frame_id = 'intermediate'
        world_frame_id = 'world'

        converter = OdometryConverter(
            self, frame_id_in, frame_id_out, broadcast_tf, body_frame_id, intermediate_frame_id, world_frame_id)
        
        # Set up the publishers/subscribers
        converter.in_odom_sub = self.create_subscription(
            Odometry, 'in_odom', converter.in_odom_callback, 10)
        converter.out_odom_pub = self.create_publisher(
            Odometry, 'out_odom', 10)
        converter.out_path_pub = self.create_publisher(
            Path, 'out_path', 10)

        # Set up timers
        tf_timer = self.create_timer(0.1, converter.tf_pub_callback)
        path_timer = self.create_timer(0.5, converter.path_pub_callback)
        
        # Keep references to timers
        self.timers.append(tf_timer)
        self.timers.append(path_timer)

        self.converters.append(converter)
        self.get_logger().info('TF Assist node initialized with 1 converter')


def main(args=None):
    rclpy.init(args=args)
    node = TfAssist()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()