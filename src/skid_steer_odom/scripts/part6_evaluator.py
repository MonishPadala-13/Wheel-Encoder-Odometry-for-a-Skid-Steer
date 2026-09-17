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

class Part6Evaluator(Node):
    def __init__(self):
        super().__init__('part6_evaluator')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/wheel_odom', self.odom_cb, 10)
        self.gt_sub = self.create_subscription(Odometry, '/ground_truth', self.gt_cb, 10)

        self.odom_pose = None
        self.gt_pose = None

        self.timer = self.create_timer(0.05, self.loop)

        self.directions = ['CW'] * 5 + ['CCW'] * 5
        self.run_idx = 0
        self.side_idx = 0
        self.substate = 'START_LEG'

        self.leg_start_x = 0.0
        self.leg_start_y = 0.0
        self.turn_start_yaw = 0.0

        self.rest_ticks = 0
        self.max_err = 0.0

        self.cw_errors = []
        self.ccw_errors = []

        log_dir = os.path.expanduser('~/ros2_ws')
        self.csv_path = os.path.join(log_dir, 'part6_systematic_results.csv')
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['run', 'direction', 'final_x_err', 'final_y_err', 'pos_err', 'yaw_err_deg', 'max_pos_err'])

        self.get_logger().info("Part 6 Evaluator Ready. Waiting for odometry and ground truth...")

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
            self.report_summary()
            rclpy.shutdown()
            return

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

            record = {
                'x_err': x_err,
                'y_err': y_err,
                'pos_err': pos_err,
                'yaw_err': yaw_err,
                'max_err': self.max_err
            }

            if current_dir == 'CW':
                self.cw_errors.append(record)
            else:
                self.ccw_errors.append(record)

            self.get_logger().info(
                f"[Part 6 | Run {self.run_idx+1}/10 ({current_dir})] "
                f"Pos Err: {pos_err:.4f} m | Yaw Err: {yaw_err:.2f} deg | Max Err: {self.max_err:.4f} m"
            )

            self.csv_writer.writerow([self.run_idx + 1, current_dir, x_err, y_err, pos_err, yaw_err, self.max_err])
            self.csv_file.flush()

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

    def report_summary(self):
        print("\n=======================================================")
        print("         PART 6 SYSTEMATIC ERROR TEST RESULTS          ")
        print("=======================================================")
        
        avg_cw_pos = sum(r['pos_err'] for r in self.cw_errors) / len(self.cw_errors)
        avg_cw_yaw = sum(r['yaw_err'] for r in self.cw_errors) / len(self.cw_errors)
        max_cw_err = max(r['max_err'] for r in self.cw_errors)

        avg_ccw_pos = sum(r['pos_err'] for r in self.ccw_errors) / len(self.ccw_errors)
        avg_ccw_yaw = sum(r['yaw_err'] for r in self.ccw_errors) / len(self.ccw_errors)
        max_ccw_err = max(r['max_err'] for r in self.ccw_errors)

        print(f"CLOCKWISE (CW) [5 runs]:")
        print(f"  Average Final Pos Error: {avg_cw_pos:.4f} m")
        print(f"  Average Heading Error:   {avg_cw_yaw:.2f} deg")
        print(f"  Worst-case Max Err:      {max_cw_err:.4f} m\n")

        print(f"COUNTER-CLOCKWISE (CCW) [5 runs]:")
        print(f"  Average Final Pos Error: {avg_ccw_pos:.4f} m")
        print(f"  Average Heading Error:   {avg_ccw_yaw:.2f} deg")
        print(f"  Worst-case Max Err:      {max_ccw_err:.4f} m")
        print("=======================================================\n")

def main(args=None):
    rclpy.init(args=args)
    node = Part6Evaluator()
    rclpy.spin(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()