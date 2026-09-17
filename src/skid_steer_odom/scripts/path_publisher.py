#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')

        self.odom_sub = self.create_subscription(Odometry, '/wheel_odom', self.odom_cb, 10)
        self.gt_sub = self.create_subscription(Odometry, '/ground_truth', self.gt_cb, 10)

        self.odom_path_pub = self.create_publisher(Path, '/path_odom', 10)
        self.gt_path_pub = self.create_publisher(Path, '/path_ground_truth', 10)

        self.odom_path = Path()
        self.odom_path.header.frame_id = 'odom'

        self.gt_path = Path()
        self.gt_path.header.frame_id = 'world'

    def odom_cb(self, msg: Odometry):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.odom_path.poses.append(pose)
        self.odom_path.header.stamp = msg.header.stamp
        self.odom_path_pub.publish(self.odom_path)

    def gt_cb(self, msg: Odometry):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.gt_path.poses.append(pose)
        self.gt_path.header.stamp = msg.header.stamp
        self.gt_path_pub.publish(self.gt_path)

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()