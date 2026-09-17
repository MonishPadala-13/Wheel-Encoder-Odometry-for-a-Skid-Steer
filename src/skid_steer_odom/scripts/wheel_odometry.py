#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry_node')

        # Declare parameters (loaded from YAML)
        self.declare_parameter('wheel_radius', 0.08)
        self.declare_parameter('track_width', 0.45)
        self.declare_parameter('ticks_per_rev', 1024)
        self.declare_parameter('slip_factor_chi', 1.0)

        self.r = self.get_parameter('wheel_radius').value
        self.B = self.get_parameter('track_width').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.chi = self.get_parameter('slip_factor_chi').value

        self.meters_per_tick = (2.0 * math.pi * self.r) / self.ticks_per_rev

        # Pose state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_ticks = None
        self.last_time = None

        # ROS Subscriptions, Publications & TF
        self.subscription = self.create_subscription(
            Int32MultiArray, '/wheel_ticks', self.ticks_callback, 10
        )
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Wheel Odometry Node Initialized.")

    def ticks_callback(self, msg: Int32MultiArray):
        current_time = self.get_clock().now()
        ticks_fl, ticks_fr, ticks_rl, ticks_rr = msg.data

        if self.last_ticks is None:
            self.last_ticks = (ticks_fl, ticks_fr, ticks_rl, ticks_rr)
            self.last_time = current_time
            return

        # Compute delta ticks
        dt_fl = ticks_fl - self.last_ticks[0]
        dt_fr = ticks_fr - self.last_ticks[1]
        dt_rl = ticks_rl - self.last_ticks[2]
        dt_rr = ticks_rr - self.last_ticks[3]

        self.last_ticks = (ticks_fl, ticks_fr, ticks_rl, ticks_rr)

        # Time step dt
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        if dt <= 0.0:
            return

        # Convert tick increments to meters
        d_fl = dt_fl * self.meters_per_tick
        d_fr = dt_fr * self.meters_per_tick
        d_rl = dt_rl * self.meters_per_tick
        d_rr = dt_rr * self.meters_per_tick

        # Side averaging
        d_L = (d_fl + d_rl) / 2.0
        d_R = (d_fr + d_rr) / 2.0

        # Kinematic equations (Part 2)
        delta_s = (d_R + d_L) / 2.0
        B_eff = self.chi * self.B
        delta_theta = (d_R - d_L) / B_eff

        # Velocities
        v = delta_s / dt
        omega = delta_theta / dt

        # Midpoint Integration method
        theta_mid = self.theta + (delta_theta / 2.0)
        self.x += delta_s * math.cos(theta_mid)
        self.y += delta_s * math.sin(theta_mid)
        self.theta += delta_theta

        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Publish Odometry Message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        qx = 0.0
        qy = 0.0
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = omega

        self.odom_pub.publish(odom_msg)

        # Broadcast TF odom -> base_footprint
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()