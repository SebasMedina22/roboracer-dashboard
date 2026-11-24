"""Tests for chocapinto_eval.metrics on the real cones world (no ROS needed).

Run: cd src/chocapinto_eval && python3 -m pytest -q test/
"""
import json
import math
import os

import numpy as np
import pytest
from scipy.spatial import Delaunay

from chocapinto_eval.metrics import (Footprint, MonitorParams, RunMonitor, footprint_distance,
                                     load_cones_from_world, parse_cones, relative_pose)

HERE = os.path.dirname(__file__)
WORLD = os.path.expanduser(os.environ.get(
    'RRSTACK_WORLD',
    '~/rrstack/ros_ws/src/roboracer_gazebo/worlds/flw_cone_track_from_walls.world'))


# ---------------------------------------------------------------- fixtures
@pytest.fixture(scope='module')
def cones():
    with open(os.path.join(HERE, 'data', 'cones_world.json'), encoding='utf-8') as f:
        data = json.load(f)['cones']
    xy = np.array([[c['x'], c['y']] for c in data])
    return xy, [c['color'] for c in data], [c['name'] for c in data]


@pytest.fixture(scope='module')
def centre_path(cones):
    """Track centreline: blue-yellow Delaunay midpoints, chained from (0, 0) heading +x."""
    xy, colors, _ = cones
    col = np.array([c == 'blue' for c in colors])
    tri = Delaunay(xy)
    edges = {(min(i, j), max(i, j)) for s in tri.simplices
             for i, j in ((s[0], s[1]), (s[1], s[2]), (s[0], s[2]))}
    mids = np.array([(xy[i] + xy[j]) / 2 for i, j in edges
                     if col[i] != col[j] and 0.8 <= np.linalg.norm(xy[i] - xy[j]) <= 2.2])
    cur, heading = np.zeros(2), np.array([1.0, 0.0])
    left, out = list(range(len(mids))), [np.zeros(2)]
    while left:
        cand = [k for k in left
                if np.linalg.norm(mids[k] - cur) < 1.5 and np.dot(mids[k] - cur, heading) > -0.05]
        if not cand:
            break
        k = min(cand, key=lambda k: np.linalg.norm(mids[k] - cur))
        d = mids[k] - cur
        if np.linalg.norm(d) > 1e-3:
            heading = d / np.linalg.norm(d)
        cur = mids[k]
        out.append(cur)
        left = [m for m in left if m != k and np.linalg.norm(mids[m] - cur) > 0.05]
    path = np.array(out)
    path = path[np.r_[True, np.linalg.norm(np.diff(path, axis=0), axis=1) > 1e-3]]
    return path


def drive(path, laps, speed=1.0, dt=0.05):
    """Poses along a closed path at constant speed: list of (t, (x, y, yaw), s)."""
    closed = np.vstack([path, path[:1]])
    seg = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    s_cum = np.r_[0.0, np.cumsum(seg)]
    length = s_cum[-1]
    out, t = [], 0.0
    while speed * t <= laps * length + 0.5:
        s = (speed * t) % length
        x, y = np.interp(s, s_cum, closed[:, 0]), np.interp(s, s_cum, closed[:, 1])
        s2 = (s + 0.2) % length
        x2, y2 = np.interp(s2, s_cum, closed[:, 0]), np.interp(s2, s_cum, closed[:, 1])
        out.append((t, (float(x), float(y), math.atan2(y2 - y, x2 - x)), speed * t))
        t += dt
    return out, length


# ---------------------------------------------------------------- world parsing
def test_parse_inline_world():
    text = """
    <include><name>sun</name><uri>model://sun</uri></include>
    <include><name>b1</name><uri>model://blue_cone</uri><pose>1.0 2.0 0 0 0 0</pose></include>
    <include>
      <name>y1</name>
      <uri>model://yellow_cone</uri>
      <pose>-3.5 0.25 0 0 0 1.57</pose>
    </include>
    <include><name>wall</name><uri>model://flw_track</uri><pose>0 0 0 0 0 0</pose></include>
    """
    xy, colors, names = parse_cones(text)
    assert colors == ['blue', 'yellow']
    assert names == ['b1', 'y1']
    assert np.allclose(xy, [[1.0, 2.0], [-3.5, 0.25]])


def test_fixture_matches_track(cones):
    xy, colors, _ = cones
    assert len(xy) == 279
    assert colors.count('blue') == 134 and colors.count('yellow') == 145


@pytest.mark.skipif(not os.path.exists(WORLD), reason='rrstack world file not available')
def test_fixture_matches_world_file(cones):
    xy, colors, names = load_cones_from_world(WORLD)
    assert names == cones[2] and colors == cones[1]
    assert np.allclose(xy, cones[0], atol=1e-3)


# ---------------------------------------------------------------- geometry
def test_footprint_distance():
    fp = Footprint()
    pose = (0.0, 0.0, 0.0)
    pts = np.array([[0.505, 0.0],    # 0.1 ahead of the front
                    [0.2, 0.0],      # inside
                    [0.2, 0.25],     # 0.1 left of the side
                    [-0.185, 0.0],   # 0.1 behind
                    [0.505, 0.25]])  # diagonal from the front-left corner
    d = footprint_distance(pose, pts, fp)
    assert np.allclose(d, [0.1, 0.0, 0.1, 0.1, math.hypot(0.1, 0.1)])
    # same point, car turned 90 deg left: "ahead" is now +y
    d2 = footprint_distance((0.0, 0.0, math.pi / 2), np.array([[0.0, 0.505]]), fp)
    assert d2[0] == pytest.approx(0.1)


def test_relative_pose():
    rel = relative_pose((1.0, 1.0, math.pi / 2), (1.0, 3.0, math.pi / 2))
    assert rel == pytest.approx((2.0, 0.0, 0.0))


# ---------------------------------------------------------------- full runs
def test_three_clean_laps(cones, centre_path):
    poses, length = drive(centre_path, laps=3.2)
    mon = RunMonitor(*cones, params=MonitorParams(laps_target=3))
    for t, pose, _ in poses:
        mon.update(t, pose, odom=pose)
        if mon.done:
            break
    s = mon.summary()
    assert s['end_reason'] == 'laps_completed'
    assert s['laps_completed'] == 3
    assert s['contacts'] == 0
    assert s['min_clearance'] > 0.15
    for lap in s['laps']:
        assert lap['time'] == pytest.approx(length, abs=0.2)     # 1 m/s
        assert lap['avg_speed'] == pytest.approx(1.0, abs=0.01)
        assert lap['drift_xy'] == pytest.approx(0.0, abs=1e-6)  # odom == truth here


def test_contact_detected_at_the_right_cone(cones, centre_path):
    xy, colors, names = cones
    poses, _ = drive(centre_path, laps=1.0)
    target = next(k for k in range(len(xy))
                  if colors[k] == 'blue' and 5.0 < xy[k, 0] < 6.0 and xy[k, 1] > 0)
    mon = RunMonitor(*cones)
    for t, (x, y, yaw), s in poses:
        # drift 0.6 m to the left between s = 3 m and s = 5.5 m (blue cones at +0.7 m)
        lateral = 0.6 * min(max((s - 3.0) / 2.5, 0.0), 1.0)
        pose = (x - lateral * math.sin(yaw), y + lateral * math.cos(yaw), yaw)
        mon.update(t, pose)
        if mon.done:
            break
    s = mon.summary()
    assert s['end_reason'] == 'contact'
    assert s['contacts'] >= 1
    ev = s['contact_events'][0]
    assert 'blue' in ev['detail']
    assert abs(ev['x'] - xy[target, 0]) < 1.0


def test_contacts_counted_once_while_touching(cones):
    xy, colors, names = cones
    k = 0
    cone = xy[k]
    mon = RunMonitor(*cones, params=MonitorParams(stop_on_contact=False, laps_target=99))
    # park with the front touching one cone for 2 s, then back off
    touch = (cone[0] - 0.45, cone[1], 0.0)
    for i in range(41):
        mon.update(i * 0.05, touch)
    mon.update(2.1, (cone[0] - 1.5, cone[1], 0.0))
    mon.update(2.2, touch)
    assert mon.summary()['contacts'] == 2


def test_stuck_detected(cones, centre_path):
    poses, _ = drive(centre_path, laps=0.2)          # ~22 m
    mon = RunMonitor(*cones, params=MonitorParams(stop_on_contact=False))
    t_end, last = 0.0, None
    for t, pose, s in poses:
        if s > 10.0:
            break
        mon.update(t, pose)
        t_end, last = t, pose
    t = t_end
    while not mon.done and t < t_end + 20:
        t += 0.05
        mon.update(t, last)
    s = mon.summary()
    assert s['end_reason'] == 'stuck'
    stuck_t = next(e.t for e in mon.events if e.kind == 'stuck')
    assert 5.0 <= stuck_t - t_end <= 6.0


def test_not_stuck_before_moving(cones):
    mon = RunMonitor(*cones, params=MonitorParams(max_sim_time=20.0))
    for i in range(300):                               # 15 s standing at the start
        mon.update(i * 0.05, (0.0, 0.0, 0.0))
    assert not mon.done


def test_odometry_drift(cones, centre_path):
    poses, _ = drive(centre_path, laps=0.5)
    mon = RunMonitor(*cones, params=MonitorParams(stop_on_contact=False, laps_target=99))
    for t, pose, s in poses:
        odom = (pose[0] + 0.01 * s, pose[1], pose[2])    # 1 cm of error per metre, along x
        mon.update(t, pose, odom=odom)
    s_end = poses[-1][2]
    assert mon.summary()['odom_drift_xy'] == pytest.approx(0.01 * s_end, rel=0.01)
    assert mon.summary()['odom_drift_yaw_deg'] == pytest.approx(0.0, abs=1e-6)


def test_lap_needs_minimum_length(cones):
    mon = RunMonitor(*cones)
    # wiggle back and forth across the start line: never a lap
    for i in range(200):
        x = 0.3 * math.sin(i * 0.2)
        mon.update(i * 0.05, (x, 0.0, 0.0))
    assert mon.summary()['laps_completed'] == 0


def test_sim_time_limit(cones, centre_path):
    poses, _ = drive(centre_path, laps=1.0)
    mon = RunMonitor(*cones, params=MonitorParams(max_sim_time=30.0))
    for t, pose, _ in poses:
        mon.update(t, pose)
        if mon.done:
            break
    assert mon.summary()['end_reason'] == 'sim_time_limit'
    assert mon.summary()['sim_time'] == pytest.approx(30.0, abs=0.06)


def test_external_finish_and_mode_events(cones, centre_path):
    poses, _ = drive(centre_path, laps=0.1)
    mon = RunMonitor(*cones)
    for t, pose, _ in poses[:50]:
        mon.update(t, pose)
    mon.add_mode(2.5, 'track')
    mon.finish(2.5, 'wall_time_limit')
    s = mon.summary()
    assert s['end_reason'] == 'wall_time_limit'
    assert [e.detail for e in mon.events if e.kind == 'mode'] == ['track']
    json.dumps(s)                                      # summary must be JSON-serialisable
