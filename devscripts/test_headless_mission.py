#!/usr/bin/env python3
"""Execute the real Nav2 kilometre itinerary in an isolated headless simulation.

Source the built ROS workspace first. Acceleration changes only the generated
world's real-time factor; rover speeds, physics step, and sensing limits stay
unchanged. Status and evidence are checkpointed in the output directory.
"""
import argparse
import hashlib
import json
import math
import os
import signal
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--rate', type=float, default=4.)
    parser.add_argument('--domain', type=int, default=78)
    parser.add_argument('--render-backend', choices=('egl', 'display'), default='egl',
                        help='EGL without a display, or server-only rendering through WSLg/X')
    parser.add_argument('--timeout', type=float, default=7200., help='Maximum wall seconds')
    args = parser.parse_args()
    if not (math.isfinite(args.rate) and 0 < args.rate <= 10 and 1 <= args.domain <= 101
            and math.isfinite(args.timeout) and args.timeout > 0):
        parser.error('Expected rate in (0,10], domain in [1,101], and positive finite timeout')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    token = 'headless-' + uuid.uuid4().hex[:12]
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['GZ_PARTITION'] = token
    import rclpy
    from ament_index_python.packages import get_package_share_directory as share
    from controller_manager_msgs.srv import ListControllers
    from lifecycle_msgs.srv import GetState
    from nav2_msgs.msg import CollisionMonitorState
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan, NavSatFix, PointCloud2
    from std_msgs.msg import String
    from storm_dashboard.core import terminate

    world_source = Path(share('storm_gazebo')) / 'worlds/urc_mission.sdf'
    mission = Path(share('storm_mission')) / 'missions/mdrs_long_route.json'
    tree = ET.parse(world_source)
    tree.find('./world/physics/real_time_factor').text = str(args.rate)
    world = output / 'world.sdf'
    tree.write(world, encoding='unicode', xml_declaration=True)
    (output / 'waypoints.json').write_bytes(mission.read_bytes())
    summary = {'status': 'starting', 'mission_id': token, 'ros_domain': args.domain,
               'gz_partition': token, 'requested_real_time_factor': args.rate,
               'render_backend': args.render_backend,
               'graphics_environment': {key: os.environ.get(key) for key in
                                        ('GALLIUM_DRIVER', 'MESA_D3D12_DEFAULT_ADAPTER_NAME')},
               'physics_step_s': float(tree.find('./world/physics/max_step_size').text),
               'source_world_sha256': hashlib.sha256(world_source.read_bytes()).hexdigest(),
               'mission_sha256': hashlib.sha256(mission.read_bytes()).hexdigest(),
               'scope': 'Truth-localized Nav2 simulation; no payload, RF, or physical GNSS qualification',
               'message_counts': {}, 'collision_actions': {}, 'states': {}}
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('storm_headless_validation')
    latest, seen, futures = {}, {}, {}
    started = time.monotonic()
    mission_start = None
    mission_clock_start = None
    stopped = False
    simulation = process = None
    logs = []

    def stop(*_):
        nonlocal stopped
        stopped = True

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop)

    def receive(name, msg):
        latest[name] = msg
        seen[name] = time.monotonic()
        counts = summary['message_counts']
        counts[name] = counts.get(name, 0) + 1

    for name, typ, topic in (
        ('truth', Odometry, '/simulation/ground_truth/odom'),
        ('clock', Clock, '/clock'), ('objects', PointCloud2, '/perception/obstacles'),
        ('lidar', PointCloud2, '/velodyne_points'), ('scan', LaserScan, '/scan'),
        ('gnss', NavSatFix, '/gps/fix'),
    ):
        node.create_subscription(typ, topic, lambda msg, name=name: receive(name, msg), qos_profile_sensor_data)

    def collision(msg):
        key = f'{msg.action_type}:{msg.polygon_name}'
        actions = summary['collision_actions']
        actions[key] = actions.get(key, 0) + 1

    node.create_subscription(CollisionMonitorState, '/collision_monitor_state', collision, 10)
    cancel = node.create_publisher(String, '/mission/cancel',
                                  QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    names = ('controller_server', 'planner_server', 'bt_navigator', 'collision_monitor')
    clients = {name: node.create_client(GetState, '/' + name + '/get_state') for name in names}
    drive = node.create_client(ListControllers, '/controller_manager/list_controllers')

    def checkpoint():
        now = time.monotonic()
        summary['wall_elapsed_s'] = now - started
        if mission_start is not None:
            summary['mission_wall_s'] = now - mission_start
            if 'clock' in latest:
                stamp = latest['clock'].clock
                duration = stamp.sec + stamp.nanosec * 1e-9 - mission_clock_start
                summary['mission_simulation_s'] = duration
                summary['achieved_real_time_factor'] = duration / max(now - mission_start, .001)
        if 'truth' in latest:
            p = latest['truth'].pose.pose.position
            summary['truth_xy'] = [p.x, p.y]
            summary['distance_from_origin_m'] = math.hypot(p.x, p.y)
        report_path = output / 'mission.json'
        if report_path.exists():
            try:
                report = json.loads(report_path.read_text())
                summary['completed_stops'] = [w['id'] for w in report['waypoints'] if w['status'] == 'succeeded']
                summary['current_target'] = next((w['id'] for w in report['waypoints'] if w['status'] == 'running'), None)
            except (OSError, ValueError, KeyError):
                pass
        temporary = output / 'status.tmp'
        temporary.write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
        temporary.replace(output / 'status.json')
        print(json.dumps({k: summary[k] for k in ('status', 'wall_elapsed_s', 'truth_xy',
                         'achieved_real_time_factor') if k in summary}), flush=True)

    def spawn(argv, filename):
        log = (output / filename).open('w')
        logs.append(log)
        return subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    try:
        # Do not join a domain that already has rover publishers.
        discovery = time.monotonic() + 2
        while time.monotonic() < discovery:
            rclpy.spin_once(node, timeout_sec=.1)
        if any(node.count_publishers(topic) for topic in ('/clock', '/odom', '/cmd_vel')):
            raise RuntimeError('Selected ROS domain already has rover publishers; choose another domain')
        simulation = spawn(['ros2', 'launch', 'storm_bringup', 'simulation.launch.py',
                            'environment:=urc_mission', 'truth_pose:=true', 'gui:=false',
                            'rviz:=false', 'world:=' + str(world),
                            'headless_rendering:=' + str(args.render_backend == 'egl').lower()], 'simulation.log')
        ready_deadline = time.monotonic() + 150
        next_poll = 0
        while time.monotonic() < ready_deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if stopped or simulation.poll() is not None:
                raise RuntimeError('Stopped or simulation exited during startup')
            now = time.monotonic()
            if now >= next_poll:
                for name, client in {**clients, 'drive': drive}.items():
                    future = futures.get(name)
                    if future and future.done():
                        response = future.result()
                        summary['states'][name] = (any(c.name == 'diff_drive_controller' and c.state == 'active'
                                                       for c in response.controller) if name == 'drive'
                                                    else response.current_state.id)
                        futures.pop(name)
                    if name not in futures and client.service_is_ready():
                        futures[name] = client.call_async(ListControllers.Request() if name == 'drive' else GetState.Request())
                next_poll = now + .5
            if (all(summary['states'].get(name) == 3 for name in names)
                    and summary['states'].get('drive')
                    and all(now - seen.get(name, 0) < 3 for name in ('truth', 'clock', 'objects', 'lidar', 'scan', 'gnss'))):
                break
        else:
            raise RuntimeError('Headless scene did not become ready; inspect simulation.log and states')
        stamp = latest['clock'].clock
        renderer_log = Path.home() / '.gz/rendering/ogre2.log'
        if renderer_log.exists():
            summary['renderer'] = next((line.strip() for line in renderer_log.read_text(errors='replace').splitlines()
                                        if 'GL_RENDERER =' in line), 'not recorded')
        mission_clock_start = stamp.sec + stamp.nanosec * 1e-9
        mission_start = time.monotonic()
        process = spawn(['ros2', 'run', 'storm_mission', 'mission', '--mission', str(output / 'waypoints.json'),
                         '--execute', '--allow-unobserved', '--mission-id', token,
                         '--report', str(output / 'mission.json')], 'mission.log')
        summary['status'] = 'running'
        next_checkpoint = 0
        while process.poll() is None:
            rclpy.spin_once(node, timeout_sec=.05)
            now = time.monotonic()
            if stopped or now - mission_start > args.timeout:
                raise RuntimeError('Cancelled' if stopped else 'Wall-time deadline exceeded')
            if simulation.poll() is not None:
                raise RuntimeError('Simulation exited during mission')
            if now >= next_checkpoint:
                checkpoint()
                next_checkpoint = now + 15
        report = json.loads((output / 'mission.json').read_text())
        summary['mission_report'] = report
        if process.returncode or report.get('status') != 'succeeded':
            raise RuntimeError(report.get('reason', 'Mission exited unsuccessfully'))
        if len(report['waypoints']) != 3 or any(w['status'] != 'succeeded' for w in report['waypoints']):
            raise RuntimeError('Not all three ordered GNSS stops succeeded')
        if summary.get('distance_from_origin_m', 0) < 999:
            # Refresh final pose after the action result has been delivered.
            for _ in range(20):
                rclpy.spin_once(node, timeout_sec=.05)
            checkpoint()
            if summary.get('distance_from_origin_m', 0) < 999:
                raise RuntimeError('Final truth position did not reach the kilometre region')
        summary['status'] = 'passed'
    except Exception as exc:  # noqa: BLE001 -- checkpoint failure and always stop owned processes
        summary.update(status='cancelled' if stopped else 'failed', reason=str(exc))
    finally:
        if process and process.poll() is None:
            deadline = time.monotonic() + 12
            while process.poll() is None and time.monotonic() < deadline:
                cancel.publish(String(data=token))
                rclpy.spin_once(node, timeout_sec=.1)
            if process.poll() is None and simulation:
                terminate(simulation)
            terminate(process)
        checkpoint()
        if simulation:
            terminate(simulation)
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
