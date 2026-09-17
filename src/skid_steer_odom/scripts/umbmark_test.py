#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import csv
import os
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

class UMBmarkTest(Node):
    def __init__(self):
        super().__init__('umbmark_test')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/wheel_odom', self.odom_cb, 10)
        self.gt_sub = self.create_subscription(Odometry, '/ground_truth', self.gt_cb, 10)

        self.odom_pose = None
        self.gt_pose = None

        self.timer = self.create_timer(0.05, self.loop)

        # Benchmark State
        # Directions: 5 CW, then 5 CCW
        self.directions = ['CW'] * 5 + ['CCW'] * 5
        self.run_idx = 0
        self.side_idx = 0 # 0, 1, 2, 3 sides
        self.substate = 'START_LEG' # START_LEG, MOVE_STRAIGHT, START_TURN, TURN, REST

        self.leg_start_x = 0.0
        self.leg_start_y = 0.0
        self.turn_start_yaw = 0.0

        self.rest_ticks = 0
        self.max_err = 0.0

        # CSV Logging
        log_dir = os.path.expanduser('~/ros2_ws')
        self.csv_file = open(os.path.join(log_dir, 'umbmark_results.csv'), 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['run', 'direction', 'final_x_err', 'final_y_err', 'pos_err', 'yaw_err_deg', 'max_pos_err'])

        self.get_logger().info("UMBmark Test Initialized. Waiting for odometry...")

    def odom_cb(self, msg: Odometry):
        self.odom_pose = msg.pose.pose

    def gt_cb(self, msg: Odometry):
        self.gt_pose = msg.pose.pose

    def loop(self):
        if self.odom_pose is None or self.gt_pose is None:
            return

        if self.run_idx >= len(self.directions):
            self.cmd_pub.publish(Twist())
            self.csv_file.close()
            self.get_logger().info("UMBmark Test Complete! Saved to umbmark_results.csv")
            rclpy.shutdown()
            return

        # Track continuous maximum positional error
        curr_pos_err = math.hypot(
            self.odom_pose.position.x - self.gt_pose.position.x,
            self.odom_pose.position.y - self.gt_pose.position.y
        )
        if curr_pos_err > self.max_err:
            self.max_err = curr_pos_err

        current_dir = self.directions[self.run_idx]

        if self.substate == 'START_LEG':
            self.leg_start_x = self.odom_pose.position.x
            self.leg_start_y = self.odom_pose.position.y
            self.substate = 'MOVE_STRAIGHT'

        elif self.substate == 'MOVE_STRAIGHT':
            dist = math.hypot(
                self.odom_pose.position.x - self.leg_start_x,
                self.odom_pose.position.y - self.leg_start_y
            )
            if dist < 2.0:
                cmd = Twist()
                cmd.linear.x = 0.35
                self.cmd_pub.publish(cmd)
            else:
                self.cmd_pub.publish(Twist())
                self.substate = 'START_TURN'

        elif self.substate == 'START_TURN':
            self.turn_start_yaw = yaw_from_quaternion(self.odom_pose.orientation)
            self.substate = 'TURN'

        elif self.substate == 'TURN':
            curr_yaw = yaw_from_quaternion(self.odom_pose.orientation)
            turned = normalize_angle(curr_yaw - self.turn_start_yaw)

            # Target 90 degrees = pi/2
            target = -(math.pi / 2.0) if current_dir == 'CW' else (math.pi / 2.0)

            if abs(turned) < abs(target) - 0.02:
                cmd = Twist()
                cmd.angular.z = -0.5 if current_dir == 'CW' else 0.5
                self.cmd_pub.publish(cmd)
            else:
                self.cmd_pub.publish(Twist())
                self.side_idx += 1
                if self.side_idx >= 4:
                    self.substate = 'FINISH_SQUARE'
                else:
                    self.substate = 'START_LEG'

        elif self.substate == 'FINISH_SQUARE':
            odom_yaw = yaw_from_quaternion(self.odom_pose.orientation)
            gt_yaw = yaw_from_quaternion(self.gt_pose.orientation)

            x_err = self.odom_pose.position.x - self.gt_pose.position.x
            y_err = self.odom_pose.position.y - self.gt_pose.position.y
            pos_err = math.hypot(x_err, y_err)
            yaw_err = math.degrees(abs(normalize_angle(odom_yaw - gt_yaw)))

            self.get_logger().info(
                f"[Run {self.run_idx+1}/10 ({current_dir})] "
                f"Pos Err: {pos_err:.4f} m | Yaw Err: {yaw_err:.2f} deg | Max Err: {self.max_err:.4f} m"
            )

            self.csv_writer.writerow([self.run_idx + 1, current_dir, x_err, y_err, pos_err, yaw_err, self.max_err])
            self.csv_file.flush()

            # Prepare for next run
            self.side_idx = 0
            self.max_err = 0.0
            self.run_idx += 1
            self.substate = 'REST'
            self.rest_ticks = 40

        elif self.substate == 'REST':
            self.cmd_pub.publish(Twist())
            self.rest_ticks -= 1
            if self.rest_ticks <= 0:
                self.substate = 'START_LEG'

def main(args=None):
    rclpy.init(args=args)
    node = UMBmarkTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()