#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import os

class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')

        self.odom_sub = self.create_subscription(Odometry, '/wheel_odom', self.odom_cb, 10)
        self.gt_sub = self.create_subscription(Odometry, '/ground_truth', self.gt_cb, 10)

        self.odom_x, self.odom_y = [], []
        self.gt_x, self.gt_y = [], []

        self.get_logger().info("Trajectory Plotter initialized. Recording paths... Press Ctrl+C when run finishes to save plot.")

    def odom_cb(self, msg: Odometry):
        self.odom_x.append(msg.pose.pose.position.x)
        self.odom_y.append(msg.pose.pose.position.y)

    def gt_cb(self, msg: Odometry):
        self.gt_x.append(msg.pose.pose.position.x)
        self.gt_y.append(msg.pose.pose.position.y)

    def save_plot(self):
        if not self.odom_x or not self.gt_x:
            print("[TrajectoryPlotter] No data points collected. Skipping plot generation.")
            return

        plt.figure(figsize=(9, 9))
        plt.plot(self.gt_x, self.gt_y, 'g-', label='Ground Truth (Simulator Reality)', linewidth=2)
        plt.plot(self.odom_x, self.odom_y, 'r--', label='Wheel Odometry (Encoder Estimate)', linewidth=1.5)
        
        plt.scatter([0], [0], color='blue', s=100, label='Start Point (0,0)', zorder=5)
        if len(self.gt_x) > 0:
            plt.scatter([self.gt_x[-1]], [self.gt_y[-1]], color='green', marker='x', s=100, label='GT End', zorder=5)
            plt.scatter([self.odom_x[-1]], [self.odom_y[-1]], color='red', marker='x', s=100, label='Odom End', zorder=5)

        plt.title('UMBmark Square Test: Odometry vs Ground Truth Trajectory', fontsize=14)
        plt.xlabel('X (meters)', fontsize=12)
        plt.ylabel('Y (meters)', fontsize=12)
        plt.legend(loc='best')
        plt.grid(True)
        plt.axis('equal')

        out_path = os.path.expanduser('~/ros2_ws/trajectory_comparison.png')
        plt.savefig(out_path, dpi=300)
        print(f"[TrajectoryPlotter] Trajectory plot saved successfully to {out_path}")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_plot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()