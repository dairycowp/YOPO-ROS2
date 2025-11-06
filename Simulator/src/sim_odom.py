#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
import tf_transformations

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.odom_pub = self.create_publisher(Odometry, '/sim/odom', 10)
        self.timer = self.create_timer(1.0/30, self.timer_callback)  # 30 Hz

        self.x_position = 0.0
        self.y_position = 2.0
        self.z_position = 1.6
        self.velocity = 2.0  # m/s

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.start_time = self.get_clock().now()

    def timer_callback(self):
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9
        self.x_position = self.velocity * elapsed_time  # x = v * t

        # Update odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "world"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position = Point(x=self.x_position, y=self.y_position, z=self.z_position)
        q = tf_transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw)
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        odom_msg.twist.twist.linear.x = self.velocity
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        # Publish the odometry message
        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    odom_publisher = OdomPublisher()
    rclpy.spin(odom_publisher)
    odom_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()