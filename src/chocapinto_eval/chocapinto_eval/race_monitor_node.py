"""race_monitor: evaluates one simulation run and writes the results to disk.

Simulation only (it needs Gazebo's true pose, which does not exist on the real car).
It never publishes commands: it only listens.

Subscribes:
  /eval/ground_truth  tf2_msgs/TFMessage  Gazebo's dynamic_pose/info, bridged (true pose)
  /odom               nav_msgs/Odometry   pose according to wheel odometry
  /rosout             rcl_interfaces/Log  the pursuit node's "mode: ..." messages
Writes, when the run ends (laps done, contact, stuck, time limit or Ctrl+C):
  <out_dir>/<run_name>/summary.json, trajectory.csv, events.csv
"""
import csv
import json
import math
import os
import re
import time
from dataclasses import asdict
from datetime import datetime
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from tf2_msgs.msg import TFMessage

from .metrics import (MonitorParams, Pose2D, RunMonitor, load_cones_from_world,
                      yaw_from_quaternion)

MODE_RE = re.compile(r'^mode:\s*(\S+)')


def default_world_file() -> str:
    """The cones world installed by roboracer_gazebo (read only)."""
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory('roboracer_gazebo')
    return os.path.join(share, 'worlds', 'flw_cone_track_from_walls.world')


class RaceMonitorNode(Node):

    def __init__(self) -> None:
        super().__init__('race_monitor')
        p = self.declare_parameter
        p('world_file', '')
        p('car_name', 'f110_car')
        p('ground_truth_topic', '/eval/ground_truth')
        p('odom_topic', '/odom')
        p('controller_node', 'waypoint_pursuit_node')
        p('laps', 3)
        p('stop_on_contact', True)
        p('max_sim_time', 900.0)
        p('max_wall_time', 0.0)          # 0 = no limit (run_eval.sh sets one)
        p('rate', 20.0)                  # samples per second of simulation time
        p('odom_wait', 5.0)              # s of sim time to wait for /odom before starting
        p('progress_period', 10.0)       # progress line every N s of sim time (0 = off)
        p('out_dir', '~/avai_runs')
        p('run_name', '')
        g = self.get_parameter

        self.world_file = str(g('world_file').value) or default_world_file()
        self.car_name = str(g('car_name').value)
        self.controller_node = str(g('controller_node').value)
        self.max_wall_time = float(g('max_wall_time').value)
        self.odom_wait = float(g('odom_wait').value)
        self.progress_period = float(g('progress_period').value)
        run_name = str(g('run_name').value) or datetime.now().strftime('%Y%m%d_%H%M%S')
        self.run_dir = os.path.join(os.path.expanduser(str(g('out_dir').value)), run_name)

        xy, colors, names = load_cones_from_world(self.world_file)
        if len(xy) == 0:
            raise RuntimeError(f'no cones found in {self.world_file}')
        self.params = MonitorParams(
            laps_target=int(g('laps').value),
            stop_on_contact=bool(g('stop_on_contact').value),
            max_sim_time=float(g('max_sim_time').value),
        )
        self.monitor = RunMonitor(xy, colors, names, params=self.params)

        self._truth: Optional[Pose2D] = None
        self._odom: Optional[Pose2D] = None
        self._first_truth_t: Optional[float] = None
        self._wall_start = time.monotonic()
        self._started_iso = datetime.now().isoformat(timespec='seconds')
        self.finished = False
        self._progress_mark: Optional[tuple] = None    # (sim t, wall t, row index)

        self.create_subscription(TFMessage, str(g('ground_truth_topic').value),
                                 self._on_truth, qos_profile_sensor_data)
        self.create_subscription(Odometry, str(g('odom_topic').value),
                                 self._on_odom, qos_profile_sensor_data)
        self.create_subscription(Log, '/rosout', self._on_log,
                                 QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
        self.create_timer(1.0 / float(g('rate').value), self._tick)

        self.get_logger().info(
            f'race_monitor: {len(xy)} cones from {self.world_file}; '
            f'target {self.params.laps_target} laps; results -> {self.run_dir}')

    # ------------------------------------------------------------ callbacks
    def _sim_time(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_truth(self, msg: TFMessage) -> None:
        for tf in msg.transforms:
            if tf.child_frame_id == self.car_name:
                q = tf.transform.rotation
                self._truth = (tf.transform.translation.x, tf.transform.translation.y,
                               yaw_from_quaternion(q.x, q.y, q.z, q.w))
                return

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self._odom = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                      yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def _on_log(self, msg: Log) -> None:
        if not msg.name.endswith(self.controller_node):
            return
        m = MODE_RE.match(msg.msg)
        if m:
            self.monitor.add_mode(self._sim_time(), m.group(1))

    def _tick(self) -> None:
        if self.finished:
            return
        if self.max_wall_time > 0 and time.monotonic() - self._wall_start > self.max_wall_time:
            self.monitor.finish(self._sim_time(), 'wall_time_limit')
            self.finished = True
            return
        t = self._sim_time()
        if self._truth is None:
            return
        if self._first_truth_t is None:
            self._first_truth_t = t
            self.get_logger().info(f'true pose of {self.car_name} received')
        if self._odom is None and t - self._first_truth_t < self.odom_wait:
            return                                   # give /odom a moment so drift starts at 0
        for ev in self.monitor.update(t, self._truth, self._odom):
            if ev.kind in ('lap', 'contact', 'stuck', 'end'):
                self.get_logger().info(f'[{ev.kind}] t={ev.t:.1f}s ({ev.x:.2f}, {ev.y:.2f}) '
                                       f'{ev.detail}')
        self._progress(t)
        if self.monitor.done:
            self.finished = True

    def _progress(self, t: float) -> None:
        """Every progress_period s of sim time: distance, lap, speed, closest cone, RTF."""
        if self.progress_period <= 0 or not self.monitor.rows:
            return
        wall = time.monotonic()
        if self._progress_mark is None:
            self._progress_mark = (t, wall, len(self.monitor.rows))
            return
        t0, w0, i0 = self._progress_mark
        if t - t0 < self.progress_period:
            return
        rows = self.monitor.rows[i0:]
        last = self.monitor.rows[-1]
        closest = min(r['clearance'] for r in rows)
        rtf = (t - t0) / (wall - w0) if wall > w0 else float('nan')
        self.get_logger().info(
            f"[progress] sim {last['t']:.0f} s | {self.monitor.summary()['distance']:.0f} m | "
            f"lap {last['lap']}/{self.params.laps_target} | {last['speed']:.2f} m/s | "
            f'closest cone {closest:.2f} m | RTF {rtf:.2f}')
        self._progress_mark = (t, wall, len(self.monitor.rows))

    # ------------------------------------------------------------ output
    def write_results(self, reason_if_running: str = 'interrupted') -> str:
        if not self.monitor.done:
            try:
                t = self._sim_time()
            except Exception:                        # clock gone after shutdown
                t = 0.0
            self.monitor.finish(t, reason_if_running)
        if not self.monitor.done:                    # never got a single true pose
            self.monitor.done = True
            self.monitor.end_reason = ('no_true_pose' if self._first_truth_t is None
                                       else reason_if_running)
        os.makedirs(self.run_dir, exist_ok=True)
        wall = time.monotonic() - self._wall_start
        summary = self.monitor.summary()
        summary.update({
            'run_dir': self.run_dir,
            'started': self._started_iso,
            'wall_time': round(wall, 1),
            'rtf_avg': round(summary['sim_time'] / wall, 3) if wall > 0 else None,
            'world_file': self.world_file,
            'car_name': self.car_name,
            'params': asdict(self.params),
            'true_pose_received': self._first_truth_t is not None,
            'odom_received': self._odom is not None,
        })
        with open(os.path.join(self.run_dir, 'summary.json'), 'w', encoding='utf-8') as f:
            json.dump(summary, f, indent=2)
        if self.monitor.rows:
            with open(os.path.join(self.run_dir, 'trajectory.csv'), 'w', newline='',
                      encoding='utf-8') as f:
                w = csv.DictWriter(f, fieldnames=list(self.monitor.rows[0].keys()))
                w.writeheader()
                w.writerows(self.monitor.rows)
        with open(os.path.join(self.run_dir, 'events.csv'), 'w', newline='',
                  encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(['t', 'kind', 'x', 'y', 'detail'])
            t0 = self.monitor.events[0].t if self.monitor.events else 0.0
            for e in self.monitor.events:
                x = '' if math.isnan(e.x) else round(e.x, 3)
                y = '' if math.isnan(e.y) else round(e.y, 3)
                w.writerow([round(e.t - t0, 3), e.kind, x, y, e.detail])
        return summary['end_reason']


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RaceMonitorNode()
    reason = 'interrupted'
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        reason = node.monitor.end_reason or 'interrupted'
    except KeyboardInterrupt:
        reason = 'interrupted'
    finally:
        end = node.write_results(reason)
        summary_path = os.path.join(node.run_dir, 'summary.json')
        print(f'[race_monitor] run ended: {end} -> {summary_path}', flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
