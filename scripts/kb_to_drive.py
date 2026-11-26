#!/usr/bin/env python3
"""Drive the real car with teleop_twist_keyboard.

teleop_twist_keyboard publishes geometry_msgs/Twist on /cmd_vel, but the car's
base only listens to ackermann_msgs/AckermannDriveStamped on /drive (and
/teleop for the joypad). This node converts /cmd_vel -> /drive.

  linear.x  -> drive.speed           (clamped to +-max_speed, m/s)
  angular.z -> drive.steering_angle  (angular.z * max_steer, clamped)

Safety:
  * re-publishes the last command at 20 Hz (the mux drops /drive after a short
    timeout), but only for hold_s seconds after the last key; then speed 0.
  * stops (speed 0) on Ctrl+C.
  * the joypad (/teleop) has priority in the mux: holding its deadman button
    with the sticks centred overrides this node.

Usage (two terminals on the car):
  python3 scripts/kb_to_drive.py            # max 0.5 m/s
  python3 scripts/kb_to_drive.py 0.8        # other speed cap
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""
import sys
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist

MAX_STEER = 0.34    # rad, F1TENTH steering limit
HOLD_S = 0.6        # keep the last key this long (covers key-repeat delay)
RATE_HZ = 20.0


def clamp(v, lim):
    return max(-lim, min(lim, v))


def main():
    max_speed = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
    max_speed = min(max_speed, 1.5)
    rclpy.init()
    node = rclpy.create_node('kb_to_drive')
    pub = node.create_publisher(AckermannDriveStamped, '/drive', 10)
    state = {'speed': 0.0, 'steer': 0.0, 't': 0.0}

    def on_twist(msg):
        state['speed'] = clamp(msg.linear.x, max_speed)
        state['steer'] = clamp(msg.angular.z * MAX_STEER, MAX_STEER)
        state['t'] = time.monotonic()

    def publish(speed, steer):
        out = AckermannDriveStamped()
        out.header.stamp = node.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.drive.speed = float(speed)
        out.drive.steering_angle = float(steer)
        pub.publish(out)

    def tick():
        if time.monotonic() - state['t'] > HOLD_S:
            publish(0.0, 0.0)
        else:
            publish(state['speed'], state['steer'])

    node.create_subscription(Twist, '/cmd_vel', on_twist, 10)
    node.create_timer(1.0 / RATE_HZ, tick)
    node.get_logger().info(
        f'/cmd_vel -> /drive  (max {max_speed} m/s, steer +-{MAX_STEER} rad). '
        'Run teleop_twist_keyboard in another terminal.')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            publish(0.0, 0.0)
            time.sleep(0.05)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
