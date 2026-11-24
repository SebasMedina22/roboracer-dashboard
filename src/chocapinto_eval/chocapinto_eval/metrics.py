"""Evaluation of one simulation run from the car's true pose (no ROS in this file).

Inputs per update: simulation time, the car's true pose from Gazebo, and optionally its
pose according to /odom. From those the monitor derives laps and lap times, contacts
with cones, the minimum clearance to the cones, whether the car is stuck, and how far
odometry has drifted from the truth. It also decides when a run is over.

Geometry (from roboracer_gazebo, not modified):
  - car footprint relative to the model origin (= base_link): chassis box 0.49 x 0.18 m
    centred 0.16 m ahead, wheels at +-0.1055 m (+0.02 offset, 0.045 m wide)
    -> x in [-0.085, 0.405] m, y in [-0.15, 0.15] m
  - cones: cylinder collision, base radius 0.117 m
"""
import math
import re
from dataclasses import asdict, dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

Pose2D = Tuple[float, float, float]   # x, y, yaw


# ---------------------------------------------------------------- SE(2) helpers
def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def relative_pose(origin: Pose2D, pose: Pose2D) -> Pose2D:
    """pose expressed in the frame of origin (inverse(origin) * pose)."""
    dx, dy = pose[0] - origin[0], pose[1] - origin[1]
    c, s = math.cos(origin[2]), math.sin(origin[2])
    return (c * dx + s * dy, -s * dx + c * dy, wrap_angle(pose[2] - origin[2]))


# ---------------------------------------------------------------- world file
def load_cones_from_world(path: str) -> Tuple[np.ndarray, List[str], List[str]]:
    """Cone positions, colours ('blue'/'yellow') and names from a Gazebo .world file.

    Reads <include> blocks whose <uri> is model://blue_cone or model://yellow_cone.
    """
    with open(path, encoding='utf-8') as f:
        text = f.read()
    return parse_cones(text)


_INCLUDE = re.compile(r'<include>(.*?)</include>', re.S)
_TAG = {k: re.compile(rf'<{k}>\s*([^<]+?)\s*</{k}>') for k in ('name', 'uri', 'pose')}


def parse_cones(world_text: str) -> Tuple[np.ndarray, List[str], List[str]]:
    xy, colors, names = [], [], []
    for block in _INCLUDE.findall(world_text):
        uri = _TAG['uri'].search(block)
        pose = _TAG['pose'].search(block)
        if not uri or not pose:
            continue
        model = uri.group(1).rsplit('/', 1)[-1]
        if model not in ('blue_cone', 'yellow_cone'):
            continue
        vals = [float(v) for v in pose.group(1).split()]
        name = _TAG['name'].search(block)
        xy.append(vals[:2])
        colors.append(model.split('_')[0])
        names.append(name.group(1) if name else model)
    return np.array(xy, dtype=float).reshape(-1, 2), colors, names


# ---------------------------------------------------------------- footprint
@dataclass
class Footprint:
    x_min: float = -0.085
    x_max: float = 0.405
    half_width: float = 0.15


def footprint_distance(pose: Pose2D, pts: np.ndarray, fp: Footprint) -> np.ndarray:
    """Distance from each point to the car's rectangle (0 if inside)."""
    pts = np.asarray(pts, dtype=float).reshape(-1, 2)
    c, s = math.cos(pose[2]), math.sin(pose[2])
    dx, dy = pts[:, 0] - pose[0], pts[:, 1] - pose[1]
    lx, ly = c * dx + s * dy, -s * dx + c * dy           # points in the car frame
    ex = np.maximum(np.maximum(fp.x_min - lx, lx - fp.x_max), 0.0)
    ey = np.maximum(np.abs(ly) - fp.half_width, 0.0)
    return np.hypot(ex, ey)


# ---------------------------------------------------------------- run monitor
@dataclass
class MonitorParams:
    laps_target: int = 3
    stop_on_contact: bool = True
    cone_radius: float = 0.117
    min_lap_length: float = 30.0      # a lap only counts after this many metres [m]
    start_half_width: float = 2.5     # start line extends this far left/right [m]
    start_heading_tol: float = 0.8    # car must cross roughly forwards [rad]
    stuck_speed: float = 0.05         # [m/s]
    stuck_time: float = 5.0           # [s of simulation time]
    moving_speed: float = 0.2         # the stuck check starts once the car exceeds this [m/s]
    max_sim_time: float = 900.0       # safety stop [s of simulation time]
    speed_window: float = 0.5         # speed is measured over this time window [s]


@dataclass
class Event:
    t: float
    kind: str                         # start, lap, contact, stuck, mode, end
    x: float
    y: float
    detail: str = ''


@dataclass
class LapStats:
    lap: int
    time: float
    length: float
    avg_speed: float
    max_speed: float
    min_clearance: float
    contacts: int
    drift_xy: Optional[float]
    drift_yaw_deg: Optional[float]


class RunMonitor:
    """Feed it updates in time order; read events, rows and summary()."""

    def __init__(self, cones_xy: np.ndarray, colors: Sequence[str], names: Sequence[str],
                 params: MonitorParams = MonitorParams(), footprint: Footprint = Footprint()):
        self.cones = np.asarray(cones_xy, dtype=float).reshape(-1, 2)
        self.colors = list(colors)
        self.names = list(names)
        self.p = params
        self.fp = footprint

        self.events: List[Event] = []
        self.rows: List[Dict[str, float]] = []
        self.laps: List[LapStats] = []
        self.done = False
        self.end_reason = ''

        self._t0: Optional[float] = None
        self._start: Optional[Pose2D] = None
        self._odom0: Optional[Pose2D] = None
        self._last: Optional[Tuple[float, Pose2D]] = None
        self._history: List[Tuple[float, float]] = []        # (t, cumulative distance)
        self._distance = 0.0
        self._in_contact: set = set()
        self._armed = False
        self._slow_since: Optional[float] = None
        self._drift = (None, None)
        # per-lap accumulators
        self._lap_t0 = 0.0
        self._lap_d0 = 0.0
        self._lap_vmax = 0.0
        self._lap_clear = math.inf
        self._lap_contacts = 0
        self._total_contacts = 0
        self._min_clear = math.inf
        self._vmax = 0.0

    # ------------------------------------------------------------ main entry
    def update(self, t: float, pose: Pose2D, odom: Optional[Pose2D] = None) -> List[Event]:
        """One sample. t: simulation time [s]; pose: true pose; odom: pose from /odom."""
        if self.done:
            return []
        new: List[Event] = []
        if self._t0 is None:
            self._t0, self._start, self._odom0 = t, pose, odom
            self._last = (t, pose)
            self._lap_t0 = t
            self._history = [(t, 0.0)]
            new.append(Event(t, 'start', pose[0], pose[1]))
            self.events += new
            return new

        t_prev, prev = self._last
        step = math.hypot(pose[0] - prev[0], pose[1] - prev[1])
        self._distance += step
        self._history.append((t, self._distance))
        while len(self._history) > 2 and t - self._history[1][0] >= self.p.speed_window:
            self._history.pop(0)
        dt = t - self._history[0][0]
        speed = (self._distance - self._history[0][1]) / dt if dt > 1e-6 else 0.0
        self._lap_vmax = max(self._lap_vmax, speed)
        self._vmax = max(self._vmax, speed)

        # clearance and contacts
        clear = footprint_distance(pose, self.cones, self.fp) - self.p.cone_radius
        k_min = int(np.argmin(clear))
        c_min = float(clear[k_min])
        self._lap_clear = min(self._lap_clear, c_min)
        self._min_clear = min(self._min_clear, c_min)
        touching = set(np.nonzero(clear < 0.0)[0].tolist())
        for k in sorted(touching - self._in_contact):
            self._lap_contacts += 1
            self._total_contacts += 1
            new.append(Event(t, 'contact', float(self.cones[k, 0]), float(self.cones[k, 1]),
                             f'{self.names[k]} ({self.colors[k]})'))
        self._in_contact = touching

        # odometry drift: motion since the start in each frame
        if odom is not None and self._odom0 is not None:
            true_rel = relative_pose(self._start, pose)
            odom_rel = relative_pose(self._odom0, odom)
            self._drift = (math.hypot(true_rel[0] - odom_rel[0], true_rel[1] - odom_rel[1]),
                           math.degrees(abs(wrap_angle(true_rel[2] - odom_rel[2]))))

        # laps
        lap_length = self._distance - self._lap_d0
        if self._crossed_start(prev, pose) and lap_length >= self.p.min_lap_length:
            stats = self._close_lap(t)
            new.append(Event(t, 'lap', pose[0], pose[1],
                             f'lap {stats.lap}: {stats.time:.2f} s, {stats.avg_speed:.2f} m/s'))

        # stuck
        if speed > self.p.moving_speed:
            self._armed = True
        if self._armed and speed < self.p.stuck_speed:
            self._slow_since = t if self._slow_since is None else self._slow_since
            if t - self._slow_since >= self.p.stuck_time:
                detail = f'{t - self._slow_since:.1f} s below {self.p.stuck_speed} m/s'
                new.append(Event(t, 'stuck', pose[0], pose[1], detail))
                self._finish(t, pose, 'stuck', new)
        else:
            self._slow_since = None

        # other end conditions
        if not self.done:
            if len(self.laps) >= self.p.laps_target:
                self._finish(t, pose, 'laps_completed', new)
            elif self.p.stop_on_contact and any(e.kind == 'contact' for e in new):
                self._finish(t, pose, 'contact', new)
            elif t - self._t0 >= self.p.max_sim_time:
                self._finish(t, pose, 'sim_time_limit', new)

        self.rows.append({
            't': round(t - self._t0, 3), 'x': pose[0], 'y': pose[1], 'yaw': pose[2],
            'odom_x': odom[0] if odom else float('nan'),
            'odom_y': odom[1] if odom else float('nan'),
            'odom_yaw': odom[2] if odom else float('nan'),
            'speed': speed, 'clearance': c_min, 'lap': len(self.laps) + 1,
            'drift_xy': self._drift[0] if self._drift[0] is not None else float('nan'),
        })
        self._last = (t, pose)
        self.events += new
        return new

    def add_mode(self, t: float, mode: str) -> None:
        """Controller mode change (e.g. from the pursuit node's 'mode: track' log)."""
        x, y = (self._last[1][0], self._last[1][1]) if self._last else (float('nan'), float('nan'))
        self.events.append(Event(t, 'mode', x, y, mode))

    def finish(self, t: float, reason: str) -> None:
        """End from outside (e.g. wall-clock limit or Ctrl+C)."""
        if not self.done and self._last is not None:
            self._finish(t, self._last[1], reason, self.events)

    # ------------------------------------------------------------ internals
    def _crossed_start(self, prev: Pose2D, pose: Pose2D) -> bool:
        s = self._start
        a, b = relative_pose(s, prev), relative_pose(s, pose)
        return (a[0] < 0.0 <= b[0] and abs(b[1]) < self.p.start_half_width
                and abs(b[2]) < self.p.start_heading_tol)

    def _close_lap(self, t: float) -> LapStats:
        length = self._distance - self._lap_d0
        lap_time = t - self._lap_t0
        stats = LapStats(
            lap=len(self.laps) + 1, time=round(lap_time, 3), length=round(length, 2),
            avg_speed=round(length / lap_time, 3) if lap_time > 0 else 0.0,
            max_speed=round(self._lap_vmax, 3), min_clearance=round(self._lap_clear, 3),
            contacts=self._lap_contacts,
            drift_xy=None if self._drift[0] is None else round(self._drift[0], 3),
            drift_yaw_deg=None if self._drift[1] is None else round(self._drift[1], 2),
        )
        self.laps.append(stats)
        self._lap_t0, self._lap_d0 = t, self._distance
        self._lap_vmax, self._lap_clear, self._lap_contacts = 0.0, math.inf, 0
        return stats

    def _finish(self, t: float, pose: Pose2D, reason: str, sink: List[Event]) -> None:
        self.done = True
        self.end_reason = reason
        sink.append(Event(t, 'end', pose[0], pose[1], reason))

    # ------------------------------------------------------------ output
    def summary(self) -> Dict:
        sim_time = (self._last[0] - self._t0) if self._last and self._t0 is not None else 0.0
        return {
            'end_reason': self.end_reason or 'running',
            'laps_completed': len(self.laps),
            'laps_target': self.p.laps_target,
            'sim_time': round(sim_time, 2),
            'distance': round(self._distance, 2),
            'avg_speed': round(self._distance / sim_time, 3) if sim_time > 0 else 0.0,
            'max_speed': round(self._vmax, 3),
            'min_clearance': None if math.isinf(self._min_clear) else round(self._min_clear, 3),
            'contacts': self._total_contacts,
            'odom_drift_xy': None if self._drift[0] is None else round(self._drift[0], 3),
            'odom_drift_yaw_deg': None if self._drift[1] is None else round(self._drift[1], 2),
            'laps': [asdict(lap) for lap in self.laps],
            'contact_events': [asdict(e) for e in self.events if e.kind == 'contact'],
        }
