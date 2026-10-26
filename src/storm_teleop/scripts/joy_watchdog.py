#!/usr/bin/env python3
"""joy_watchdog: forwards joy/raw to joy only while the controller is alive.

joy_node keeps republishing the last state until the kernel drops a dead
Bluetooth controller. DS4/DualSense stream motion reports nonstop, so silence
on their "Motion Sensors" input device means the controller is gone: send one
all-zero Joy (deadman released) and block joy/raw until it's back.
"""

import glob
import os
import select
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


def find_motion_device():
    for name_file in sorted(glob.glob('/sys/class/input/event*/device/name')):
        try:
            with open(name_file) as f:
                if 'Motion Sensors' in f.read():
                    return '/dev/input/' + name_file.split('/')[4]
        except OSError:
            pass
    return None


class JoyWatchdog(Node):

    def __init__(self):
        super().__init__('joy_watchdog')
        self.timeout = self.declare_parameter('timeout_ms', 250).value / 1000.0
        self.require_motion = self.declare_parameter('require_motion', False).value

        self.device = None
        self.last_motion = 0.0
        self.seen_motion_device = False  # once motion data arrives, never fall back to pass-through
        self.alive = False
        self.last_joy = None

        self.pub = self.create_publisher(Joy, 'joy', 10)
        self.create_subscription(Joy, 'joy/raw', self.on_joy, 10)
        self.create_timer(0.02, self.check)

        threading.Thread(target=self.read_motion, daemon=True).start()

    def read_motion(self):
        fd = None
        while rclpy.ok():
            if fd is None:
                path = find_motion_device()
                if path is None:
                    self.device = None
                    time.sleep(1.0)
                    continue
                try:
                    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
                except OSError as e:
                    self.get_logger().warn(f'Can\'t open {path}: {e}', throttle_duration_sec=10.0)
                    time.sleep(1.0)
                    continue
                self.device = path
                self.get_logger().info(f'Watching {path}')
            try:
                ready, _, _ = select.select([fd], [], [], 0.5)
                if ready and os.read(fd, 4096):
                    self.last_motion = time.monotonic()
                    if not self.seen_motion_device:
                        self.seen_motion_device = True
                        self.get_logger().info(f'Motion data streaming from {path}')
            except OSError:
                # Device removed
                os.close(fd)
                fd = None
                self.device = None

    def motion_required(self):
        return self.require_motion or self.seen_motion_device

    def check(self):
        alive = time.monotonic() - self.last_motion < self.timeout
        if self.alive and not alive:
            self.get_logger().warn('Controller lost, stopping')
            self.publish_zero()
        elif alive and not self.alive:
            self.get_logger().info('Controller back')
        self.alive = alive

    def on_joy(self, msg):
        self.last_joy = msg
        if self.alive:
            self.pub.publish(msg)
        elif not self.motion_required():
            self.get_logger().warn(
                'No motion sensor device; only the BT failsafe and cmd_vel_timeout '
                'protect against a dead controller', throttle_duration_sec=30.0)
            self.pub.publish(msg)

    def publish_zero(self):
        if self.last_joy is None:
            return
        zero = Joy()
        zero.header.stamp = self.get_clock().now().to_msg()
        zero.header.frame_id = self.last_joy.header.frame_id
        zero.axes = [0.0] * len(self.last_joy.axes)
        zero.buttons = [0] * len(self.last_joy.buttons)
        self.pub.publish(zero)


def main():
    rclpy.init()
    node = JoyWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
