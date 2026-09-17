#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray

class EncoderEmulator(Node):
    def __init__(self):
        super().__init__('encoder_emulator')
        
        self.declare_parameter('ticks_per_rev', 1024)
        self.declare_parameter('right_wheel_scale', 1.0)
        
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.right_scale = self.get_parameter('right_wheel_scale').value

        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.ticks_pub = self.create_publisher(Int32MultiArray, '/wheel_ticks', 10)

        self.target_keys = ['front_left', 'front_right', 'rear_left', 'rear_right']
        self.matched_indices = {}

        self.get_logger().info("Encoder Emulator Node started, waiting for /joint_states...")

    def joint_callback(self, msg: JointState):
        if len(self.matched_indices) < 4:
            for key in self.target_keys:
                for i, name in enumerate(msg.name):
                    if key in name:
                        self.matched_indices[key] = i
                        break

            if len(self.matched_indices) < 4:
                return

        rad_fl = msg.position[self.matched_indices['front_left']]
        rad_fr = msg.position[self.matched_indices['front_right']]
        rad_rl = msg.position[self.matched_indices['rear_left']]
        rad_rr = msg.position[self.matched_indices['rear_right']]

        # Convert continuous radians to discrete integer ticks
        ticks_fl = int((rad_fl / (2.0 * math.pi)) * self.ticks_per_rev)
        ticks_rl = int((rad_rl / (2.0 * math.pi)) * self.ticks_per_rev)
        
        ticks_fr = int((rad_fr / (2.0 * math.pi)) * self.ticks_per_rev * self.right_scale)
        ticks_rr = int((rad_rr / (2.0 * math.pi)) * self.ticks_per_rev * self.right_scale)

        out_msg = Int32MultiArray()
        out_msg.data = [ticks_fl, ticks_fr, ticks_rl, ticks_rr]
        self.ticks_pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EncoderEmulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()