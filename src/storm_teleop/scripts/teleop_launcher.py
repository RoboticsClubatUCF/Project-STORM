#!/usr/bin/env python3
"""teleop_launcher: starts and stops teleop on request from the dashboard.

Never starts teleop by itself: not at boot, not after a crash, not when the
launcher restarts. Profiles are a fixed map so callers can't pick arbitrary
launch files or args.
"""

import os
import signal
import subprocess
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import Trigger
from storm_interfaces.msg import TeleopStatus
from storm_interfaces.srv import StartTeleop

PROFILES = {
    StartTeleop.Request.PROFILE_TELEOP: 'teleop.launch.py',
    StartTeleop.Request.PROFILE_SPACEU: 'teleop-spaceu.launch.py',
}

SIGINT_WAIT_S = 10.0
SIGTERM_WAIT_S = 5.0


class TeleopLauncher(Node):

    def __init__(self):
        super().__init__('teleop_launcher')
        self.proc = None
        self.profile = ''
        self.use_joy = False
        self.started_at = 0.0
        self.message = 'stopped'

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(TeleopStatus, '/storm/teleop/status', qos)
        self.create_service(StartTeleop, '/storm/teleop/start', self.on_start)
        self.create_service(Trigger, '/storm/teleop/stop', self.on_stop)
        self.create_timer(1.0, self.publish_status)
        self.publish_status()

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def on_start(self, req, res):
        if self.running():
            res.success = False
            res.message = f'already running ({self.profile})'
            return res
        launch_file = PROFILES.get(req.profile)
        if launch_file is None:
            res.success = False
            res.message = f'unknown profile "{req.profile}"'
            return res

        cmd = ['ros2', 'launch', 'storm_teleop', launch_file,
               f'use_joy:={"true" if req.use_joy else "false"}',
               'enable_bridge:=false', 'enable_telemetry:=false']
        self.get_logger().info('Starting: ' + ' '.join(cmd))
        try:
            # Own process group so stop can signal the whole launch tree
            self.proc = subprocess.Popen(cmd, start_new_session=True)
        except OSError as e:
            res.success = False
            res.message = f'failed to start: {e}'
            return res

        self.profile = req.profile
        self.use_joy = req.use_joy
        self.started_at = time.monotonic()
        self.message = 'running'
        self.publish_status()
        res.success = True
        res.message = f'started {req.profile}, use_joy={self.use_joy}'
        return res

    def on_stop(self, _req, res):
        if not self.running():
            res.success = True
            res.message = 'not running'
            return res
        res.message = self.stop_teleop()
        res.success = not self.running()
        self.publish_status()
        return res

    def stop_teleop(self):
        """SIGINT first so ros2_control deactivates and zeroes the motors."""
        pgid = os.getpgid(self.proc.pid)
        for sig, wait in ((signal.SIGINT, SIGINT_WAIT_S),
                          (signal.SIGTERM, SIGTERM_WAIT_S),
                          (signal.SIGKILL, SIGTERM_WAIT_S)):
            self.get_logger().info(f'Stopping teleop ({signal.Signals(sig).name})')
            try:
                os.killpg(pgid, sig)
                self.proc.wait(timeout=wait)
                break
            except subprocess.TimeoutExpired:
                continue
            except ProcessLookupError:
                break
        self.message = 'stopped' if sig == signal.SIGINT else f'stopped ({signal.Signals(sig).name})'
        return self.message

    def publish_status(self):
        running = self.running()
        if self.proc is not None and not running and self.message == 'running':
            # Exited on its own; report it, never restart
            self.message = f'exited with code {self.proc.returncode}'
            self.get_logger().warn(f'Teleop {self.message}')

        msg = TeleopStatus()
        msg.running = running
        msg.profile = self.profile
        msg.use_joy = self.use_joy
        msg.uptime_s = time.monotonic() - self.started_at if running else 0.0
        msg.message = self.message
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = TeleopLauncher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.running():
            node.stop_teleop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
