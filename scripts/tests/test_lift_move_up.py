#!/usr/bin/env python3
"""
Read current LIFT height and move up by 10cm using ROS2.

Prerequisites:
    ros2 launch arx_lift_controller lift.launch.py

Usage:
    python scripts/tests/test_lift_move_up.py
"""

import rclpy
from rclpy.node import Node
from arm_control.msg import PosCmd
import time
import sys


class LiftMoveUp(Node):
    def __init__(self):
        super().__init__('lift_move_up')

        # Publisher for control commands
        self.cmd_publisher = self.create_publisher(PosCmd, '/ARX_VR_L', 10)

        # Subscriber for state feedback
        self.body_info_sub = self.create_subscription(
            PosCmd, '/body_information', self.body_info_callback, 10)

        # State
        self.current_height = 0.0
        self.current_head_pit = 0.0
        self.current_head_yaw = 0.0
        self.body_info_received = False

        self.get_logger().info('LIFT Move Up node initialized')

    def body_info_callback(self, msg):
        """Receive robot state"""
        self.current_height = msg.height
        self.current_head_pit = msg.head_pit
        self.current_head_yaw = msg.head_yaw
        self.body_info_received = True

    def send_command(self, height):
        """Send height command (height needs to be multiplied by 41.54)"""
        msg = PosCmd()
        msg.height = height * 41.54
        msg.head_pit = self.current_head_pit
        msg.head_yaw = self.current_head_yaw
        msg.temp_float_data = [0.0] * 6
        msg.chx = 0.0
        msg.chy = 0.0
        msg.chz = 0.0
        msg.mode1 = 2
        self.cmd_publisher.publish(msg)

    def wait_for_connection(self, timeout=10.0):
        """Wait for connection to lift_controller"""
        print("Waiting for lift_controller...")
        start_time = time.time()

        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.body_info_received:
                print("  ✓ Connected")
                return True

        return False

    def run(self):
        """Main execution"""
        print("=" * 50)
        print("LIFT Control - Move Up 10cm (ROS2)")
        print("=" * 50)

        # Wait for connection
        if not self.wait_for_connection():
            print("  ✗ Failed to connect to lift_controller")
            print("  Make sure to run: ros2 launch arx_lift_controller lift.launch.py")
            return False

        # Read current height
        print(f"\nCurrent height: {self.current_height:.4f} m ({self.current_height * 100:.2f} cm)")

        # Calculate target (+10cm)
        target_height = self.current_height + 0.10
        print(f"Target height:  {target_height:.4f} m ({target_height * 100:.2f} cm)")

        # Move to target
        print(f"\nMoving LIFT up by 10cm...")
        print(f"  {self.current_height:.4f}m -> {target_height:.4f}m")

        move_duration = 3.0
        start_time = time.time()

        while time.time() - start_time < move_duration:
            self.send_command(target_height)
            rclpy.spin_once(self, timeout_sec=0.1)

            elapsed = time.time() - start_time
            if int(elapsed * 2) % 2 == 0:
                print(f"  [{elapsed:.1f}s] Height: {self.current_height:.4f}m")

        # Final reading
        rclpy.spin_once(self, timeout_sec=0.1)
        print(f"\nFinal height: {self.current_height:.4f} m ({self.current_height * 100:.2f} cm)")

        print("\n" + "=" * 50)
        print("✓ Done")
        print("=" * 50)
        return True


def main():
    rclpy.init()
    node = LiftMoveUp()

    try:
        node.run()
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
