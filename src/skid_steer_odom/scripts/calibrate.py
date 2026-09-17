#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class Calibrator(Node):
    def __init__(self):
        super().__init__('calibrator_node')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/wheel_odom', self.odom_cb, 10)
        self.gt_sub = self.create_subscription(Odometry, '/ground_truth', self.gt_cb, 10)

        self.odom_pose = None
        self.gt_pose = None

        # Angle unwrapping state
        self.last_odom_yaw = None
        self.last_gt_yaw = None
        self.unwrapped_odom_yaw = 0.0
        self.unwrapped_gt_yaw = 0.0

        self.timer = self.create_timer(0.05, self.control_loop)
        self.stage = 'WAIT_START'

        # Baseline measurements
        self.start_odom = None
        self.start_gt = None

        self.get_logger().info("Calibrator ready. Waiting for odometry and ground truth...")

    def odom_cb(self, msg: Odometry):
        self.odom_pose = msg.pose.pose
        raw_yaw = yaw_from_quaternion(self.odom_pose.orientation)
        if self.last_odom_yaw is not None:
            dyaw = raw_yaw - self.last_odom_yaw
            if dyaw > math.pi:
                dyaw -= 2.0 * math.pi
            elif dyaw < -math.pi:
                dyaw += 2.0 * math.pi
            self.unwrapped_odom_yaw += dyaw
        self.last_odom_yaw = raw_yaw

    def gt_cb(self, msg: Odometry):
        self.gt_pose = msg.pose.pose
        raw_yaw = yaw_from_quaternion(self.gt_pose.orientation)
        if self.last_gt_yaw is not None:
            dyaw = raw_yaw - self.last_gt_yaw
            if dyaw > math.pi:
                dyaw -= 2.0 * math.pi
            elif dyaw < -math.pi:
                dyaw += 2.0 * math.pi
            self.unwrapped_gt_yaw += dyaw
        self.last_gt_yaw = raw_yaw

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def control_loop(self):
        if self.odom_pose is None or self.gt_pose is None:
            return

        if self.stage == 'WAIT_START':
            self.get_logger().info("--- STARTING STEP 1: LINEAR 5-METER CALIBRATION ---")
            self.start_odom = (self.odom_pose.position.x, self.odom_pose.position.y)
            self.start_gt = (self.gt_pose.position.x, self.gt_pose.position.y)
            self.stage = 'DRIVE_STRAIGHT'

        elif self.stage == 'DRIVE_STRAIGHT':
            dx = self.odom_pose.position.x - self.start_odom[0]
            dy = self.odom_pose.position.y - self.start_odom[1]
            dist_odom = math.hypot(dx, dy)

            if dist_odom < 5.0:
                cmd = Twist()
                cmd.linear.x = 0.3
                self.cmd_pub.publish(cmd)
            else:
                self.stop_robot()
                gt_dx = self.gt_pose.position.x - self.start_gt[0]
                gt_dy = self.gt_pose.position.y - self.start_gt[1]
                dist_gt = math.hypot(gt_dx, gt_dy)

                k_s = dist_gt / dist_odom
                self.get_logger().info(f"[Distance Result] Odom: {dist_odom:.4f} m | Ground Truth: {dist_gt:.4f} m")
                self.get_logger().info(f"[Distance Result] Linear Scale Factor Correction (k_s): {k_s:.5f}")
                
                # Settle for 2 seconds before rotation test
                self.stage = 'SETTLE_BEFORE_SPIN'
                self.settle_count = 40

        elif self.stage == 'SETTLE_BEFORE_SPIN':
            self.stop_robot()
            self.settle_count -= 1
            if self.settle_count <= 0:
                self.get_logger().info("--- STARTING STEP 2: 5 FULL SPINS ROTATION CALIBRATION ---")
                self.unwrapped_odom_yaw = 0.0
                self.unwrapped_gt_yaw = 0.0
                self.stage = 'SPIN'

        elif self.stage == 'SPIN':
            # Target is 5 full spins (5 * 2 * pi)
            target_angle = 5.0 * 2.0 * math.pi
            if abs(self.unwrapped_odom_yaw) < target_angle:
                cmd = Twist()
                cmd.angular.z = 0.6
                self.cmd_pub.publish(cmd)
            else:
                self.stop_robot()
                chi = abs(self.unwrapped_odom_yaw) / max(abs(self.unwrapped_gt_yaw), 1e-6)
                self.get_logger().info(f"[Spin Result] Unwrapped Odom Yaw: {self.unwrapped_odom_yaw:.4f} rad")
                self.get_logger().info(f"[Spin Result] Unwrapped Ground Truth Yaw: {self.unwrapped_gt_yaw:.4f} rad")
                self.get_logger().info(f"======================================================")
                self.get_logger().info(f"CALIBRATED SLIP FACTOR chi: {chi:.4f}")
                self.get_logger().info(f"======================================================")
                self.stage = 'DONE'

        elif self.stage == 'DONE':
            self.stop_robot()

def main(args=None):
    rclpy.init(args=args)
    node = Calibrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()