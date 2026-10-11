#!/usr/bin/env python3
"""Keyboard teleop for the Genie G2 chassis via /cmd_4ws (forward / backward).

Run inside the container while the sim is running:
    source /workspace/devel/setup.bash
    python3 /workspace/Doku/drive_base.py [wheel_speed_rad_s]

Keys:  w = forward   s = backward   space = stop   + / - = speed   q = quit

The engine brakes the wheels when /cmd_4ws goes stale, so this script
publishes continuously (20 Hz) - also zeros while stopped.
"""

import select
import sys
import termios
import tty

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

STEER = [
    "idx111_chassis_lwheel_front_joint1",
    "idx131_chassis_rwheel_front_joint1",
    "idx121_chassis_lwheel_rear_joint1",
    "idx141_chassis_rwheel_rear_joint1",
]
DRIVE = [
    "idx112_chassis_lwheel_front_joint2",
    "idx132_chassis_rwheel_front_joint2",
    "idx122_chassis_lwheel_rear_joint2",
    "idx142_chassis_rwheel_rear_joint2",
]


def main() -> None:
    speed = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    rclpy.init()
    node = rclpy.create_node("drive_base_teleop")
    pub = node.create_publisher(JointState, "/cmd_4ws", qos_profile_sensor_data)
    direction = 0  # +1 forward, -1 backward, 0 stop

    def publish() -> None:
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = STEER + DRIVE
        msg.position = [0.0] * 8
        msg.velocity = [0.0] * 4 + [direction * speed] * 4
        pub.publish(msg)

    print("w = vorwaerts   s = rueckwaerts   Leertaste = stopp   + / - = Tempo   q = beenden")
    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.05)[0]:
                key = sys.stdin.read(1)
                if key == "w":
                    direction = 1
                elif key == "s":
                    direction = -1
                elif key == " ":
                    direction = 0
                elif key == "+":
                    speed += 1.0
                elif key == "-":
                    speed = max(0.5, speed - 1.0)
                elif key == "q":
                    break
                state = {1: "vorwaerts", -1: "rueckwaerts", 0: "stopp"}[direction]
                print(f"\r{state:12s} speed={speed:.1f} rad/s   ", end="", flush=True)
            publish()
    finally:
        direction = 0
        publish()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.shutdown()
        print()


if __name__ == "__main__":
    main()
