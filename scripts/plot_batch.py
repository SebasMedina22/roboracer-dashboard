#!/usr/bin/env python3
"""Analyse several evaluated runs together (C3): where and why the car gets close to cones.

Usage: python3 scripts/plot_batch.py RUN_DIR [RUN_DIR ...] [-o OUT_DIR] [--world WORLD_FILE]
  RUN_DIR   folders written by run_eval.sh (summary.json, trajectory.csv, events.csv)
  OUT_DIR   default: <parent of the first RUN_DIR>/analysis
Writes OUT_DIR/batch_analysis.png, batch_summary.json and batch_summary.md.

Track position s: every pose is projected onto a reference lap (the cleanest complete lap),
so all laps can be compared at the same place on the track.
Lateral offset: (distance to the yellow cone line - distance to the blue cone line) / 2,
in metres (cone lines = the cones joined in the order of their names).
0 = centred between the cone lines, < 0 = closer to the yellow (outer/right) cones.
"""
import argparse
import csv
import json
import math
import os
import statistics as st
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src', 'chocapinto_eval'))   # works without sourcing

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
except ImportError:
    sys.exit('matplotlib missing: sudo apt install -y python3-matplotlib')

from chocapinto_eval.metrics import load_cones_from_world  # noqa: E402

CLEAN = '#4c72b0'
CONTACT = '#d62728'
RUN_COLORS = ['#4c72b0', '#55a868', '#8172b2', '#c44e52', '#dd8452', '#64b5cd', '#937860']


def load_run(run_dir):
    with open(os.path.join(run_dir, 'summary.json'), encoding='utf-8') as f:
        s = json.load(f)
    with open(os.path.join(run_dir, 'trajectory.csv'), newline='', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    cols = ('t', 'x', 'y', 'yaw', 'speed', 'clearance', 'lap', 'drift_xy')
    a = np.array([[float(r[c]) if r[c] not in ('', 'nan') else math.nan for c in cols]
                  for r in rows])
    name = os.path.basename(os.path.normpath(run_dir))
    run = {'name': name, 'summary': s, 'data': {c: a[:, i] for i, c in enumerate(cols)},
           'contact_t': contact_times(run_dir)}
    # laps actually driven: complete ones, plus the lap that ended in a contact
    last = s['laps_completed'] + (1 if s['end_reason'] == 'contact' else 0)
    run['laps'] = list(range(1, last + 1))
    return run


def find_world(runs, cli_world):
    candidates = [cli_world] if cli_world else []
    candidates += [r['summary'].get('world_file') for r in runs]
    rr = os.environ.get('RRSTACK_DIR', os.path.expanduser('~/rrstack'))
    candidates.append(os.path.join(rr, 'ros_ws/src/roboracer_gazebo/worlds/'
                                       'flw_cone_track_from_walls.world'))
    for c in candidates:
        if c and os.path.exists(c):
            return c
    sys.exit('world file not found: pass --world /path/to/flw_cone_track_from_walls.world')


def nearest(points, ref, chunk=2000):
    """Index of the nearest ref point for every point (brute force, chunked)."""
    out = np.empty(len(points), dtype=int)
    for i in range(0, len(points), chunk):
        d = np.linalg.norm(points[i:i + chunk, None, :] - ref[None, :, :], axis=2)
        out[i:i + chunk] = np.argmin(d, axis=1)
    return out


def cone_line(cones, names, color):
    """Cones of one colour joined in name order (..._001, ..._002, ...), as a closed polyline."""
    idx = sorted((n, k) for k, n in enumerate(names) if n.startswith(color))
    pts = cones[[k for _, k in idx]]
    return np.vstack([pts, pts[:1]])


def dist_to_polyline(points, line):
    """Distance from every point to the nearest segment of a polyline."""
    a, b = line[:-1], line[1:]
    ab = b - a
    best = np.full(len(points), np.inf)
    for i in range(len(a)):
        t = np.clip(((points - a[i]) @ ab[i]) / max(ab[i] @ ab[i], 1e-12), 0.0, 1.0)
        best = np.minimum(best, np.linalg.norm(points - (a[i] + t[:, None] * ab[i]), axis=1))
    return best


def break_jumps(s, *ys, jump=5.0):
    """Insert NaN where s jumps (start/finish line), so lines are not drawn across."""
    cut = np.flatnonzero(np.abs(np.diff(s)) > jump) + 1
    return [np.insert(v.astype(float), cut, np.nan) for v in (s, *ys)]


def contact_times(run_dir):
    """Contact times relative to the run start, from events.csv."""
    path = os.path.join(run_dir, 'events.csv')
    if not os.path.exists(path):
        return []
    with open(path, newline='', encoding='utf-8') as f:
        return [float(e['t']) for e in csv.DictReader(f) if e['kind'] == 'contact']


def lap_mask(run, lap):
    return run['data']['lap'] == lap


def reference_lap(runs):
    """Complete lap with the largest minimum clearance (the cleanest line)."""
    best = None
    for r in runs:
        for lap in r['summary']['laps']:
            if best is None or lap['min_clearance'] > best[0]:
                best = (lap['min_clearance'], r, lap['lap'])
    _, r, lap = best
    m = lap_mask(r, lap)
    xy = np.column_stack([r['data']['x'][m], r['data']['y'][m]])
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))])
    return xy, s, f"{r['name']} lap {lap}"


def mean_sd(v):
    if not v:
        return None
    return {'mean': round(st.mean(v), 2), 'sd': round(st.stdev(v), 2) if len(v) > 1 else 0.0,
            'min': round(min(v), 2), 'max': round(max(v), 2), 'n': len(v)}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('runs', nargs='+')
    ap.add_argument('-o', '--out')
    ap.add_argument('--world')
    args = ap.parse_args()

    runs = [load_run(d) for d in args.runs]
    out = args.out or os.path.join(os.path.dirname(os.path.abspath(args.runs[0])), 'analysis')
    os.makedirs(out, exist_ok=True)
    cones, colors, names = load_cones_from_world(find_world(runs, args.world))
    blue = cones[[c == 'blue' for c in colors]]
    yellow = cones[[c == 'yellow' for c in colors]]
    blue_line = cone_line(cones, names, 'blue')
    yellow_line = cone_line(cones, names, 'yellow')

    ref_xy, ref_s, ref_name = reference_lap(runs)
    lap_len = ref_s[-1]

    # per-pose track position and lateral offset
    for r in runs:
        d = r['data']
        pts = np.column_stack([d['x'], d['y']])
        d['s'] = ref_s[nearest(pts, ref_xy)]
        d['offset'] = (dist_to_polyline(pts, yellow_line) - dist_to_polyline(pts, blue_line)) / 2.0
        d['dist'] = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(d['x']), np.diff(d['y'])))])

    # ---------------------------------------------------------------- numbers
    contacts = []
    for r in runs:
        for i, e in enumerate(r['summary']['contact_events']):
            t = r['contact_t'][i] if i < len(r['contact_t']) else e['t']
            contacts.append(dict(e, run=r['name'], t=round(t, 2)))
    lap1 = [lap['time'] for r in runs for lap in r['summary']['laps'] if lap['lap'] == 1]
    flying = [lap['time'] for r in runs for lap in r['summary']['laps'] if lap['lap'] > 1]

    # critical zone: where the lowest clearances of all laps happen
    lows = []
    for r in runs:
        d = r['data']
        for lap in r['laps']:
            m = lap_mask(r, lap)
            i = np.flatnonzero(m)[np.argmin(d['clearance'][m])]
            lows.append({'run': r['name'], 'lap': lap,
                         'clearance': round(float(d['clearance'][i]), 3),
                         'x': round(float(d['x'][i]), 2), 'y': round(float(d['y'][i]), 2),
                         's': round(float(d['s'][i]), 1), 'speed': round(float(d['speed'][i]), 2),
                         'offset': round(float(d['offset'][i]), 2),
                         'contact': bool(d['clearance'][i] <= 0)})
    s_crit = float(np.median([lo['s'] for lo in lows]))
    in_crit = [lo for lo in lows if abs(lo['s'] - s_crit) < 3.0]
    crit_x = float(np.median([lo['x'] for lo in in_crit]))
    crit_y = float(np.median([lo['y'] for lo in in_crit]))

    drift_final = [r['summary']['odom_drift_xy'] for r in runs]
    summary = {
        'runs': [r['name'] for r in runs],
        'runs_completed': sum(r['summary']['end_reason'] == 'laps_completed' for r in runs),
        'laps_target': runs[0]['summary']['laps_target'],
        'laps_completed': sum(r['summary']['laps_completed'] for r in runs),
        'contacts': [{k: c[k] for k in ('run', 't', 'detail')} for c in contacts],
        'lap1_time_s': mean_sd(lap1), 'flying_lap_time_s': mean_sd(flying),
        'odom_drift_final_m': mean_sd(drift_final),
        'reference_lap': ref_name, 'lap_length_m': round(float(lap_len), 1),
        'critical_zone': {'s': round(s_crit, 1), 'x': round(crit_x, 2), 'y': round(crit_y, 2),
                          'laps_with_min_clearance_here': f'{len(in_crit)}/{len(lows)}'},
        'lowest_clearance_per_lap': lows,
    }
    with open(os.path.join(out, 'batch_summary.json'), 'w', encoding='utf-8') as f:
        json.dump(summary, f, indent=2)

    # ---------------------------------------------------------------- figure
    fig = plt.figure(figsize=(18, 11))
    gs = fig.add_gridspec(3, 3, height_ratios=[1, 1, 1.05], hspace=0.38, wspace=0.22)
    ax_map = fig.add_subplot(gs[0:2, 0:2])
    ax_zoom = fig.add_subplot(gs[0, 2])
    ax_tab = fig.add_subplot(gs[1, 2])
    ax_clr = fig.add_subplot(gs[2, 0])
    ax_off = fig.add_subplot(gs[2, 1])
    ax_drift = fig.add_subplot(gs[2, 2])

    zoom = (crit_x - 5.0, crit_x + 5.0, crit_y - 3.0, crit_y + 5.0)
    for ax in (ax_map, ax_zoom):
        ax.scatter(blue[:, 0], blue[:, 1], s=14, c='#2f6fdb', edgecolors='k', linewidths=0.3,
                   zorder=3)
        ax.scatter(yellow[:, 0], yellow[:, 1], s=14, c='#f2c200', edgecolors='k', linewidths=0.3,
                   zorder=3)
        ax.set_aspect('equal')
        ax.grid(alpha=0.25)
    for i, r in enumerate(runs):
        d = r['data']
        had_contact = r['summary']['contacts'] > 0
        ax_map.plot(d['x'], d['y'], lw=0.9, color=RUN_COLORS[i % len(RUN_COLORS)],
                    label=f"{r['name']} ({r['summary']['laps_completed']} laps"
                          f"{', contact' if had_contact else ''})")
        for lap in r['laps']:
            m = lap_mask(r, lap)
            hit = d['clearance'][m].min() <= 0
            ax_zoom.plot(d['x'][m], d['y'][m], lw=1.6 if hit else 0.8,
                         color=CONTACT if hit else CLEAN, alpha=0.95 if hit else 0.55)
    for c in contacts:
        for ax in (ax_map, ax_zoom):
            ax.plot(c['x'], c['y'], 'x', color=CONTACT, ms=12, mew=2.5, zorder=5)
    for k, n in enumerate(names):
        if zoom[0] < cones[k, 0] < zoom[1] and zoom[2] < cones[k, 1] < zoom[3] and k % 2 == 0:
            ax_zoom.annotate(n.split('_')[-1], cones[k], fontsize=6, xytext=(2, 2),
                             textcoords='offset points')
    ax_map.add_patch(Rectangle((zoom[0], zoom[2]), zoom[1] - zoom[0], zoom[3] - zoom[2],
                               fill=False, ls='--', lw=1, color='k'))
    ax_map.plot(0, 0, 'k>', ms=10)
    ax_map.annotate('start', (0, 0), xytext=(4, -12), textcoords='offset points')
    ax_map.legend(fontsize=8, loc='lower left')
    ax_map.set_title(f'{len(runs)} runs, true trajectories (x = cone contact)')
    ax_zoom.set_xlim(zoom[0], zoom[1])
    ax_zoom.set_ylim(zoom[2], zoom[3])
    ax_zoom.set_title('Critical zone: laps with contact in red', pad=14)

    # clearance and lateral offset along the lap
    for r in runs:
        d = r['data']
        for lap in r['laps']:
            m = lap_mask(r, lap)
            hit = d['clearance'][m].min() <= 0
            sv, cv, ov = break_jumps(d['s'][m], d['clearance'][m], d['offset'][m])
            kw = dict(lw=1.3 if hit else 0.6, color=CONTACT if hit else CLEAN,
                      alpha=0.95 if hit else 0.45, zorder=3 if hit else 2)
            ax_clr.plot(sv, cv, **kw)
            ax_off.plot(sv, ov, **kw)
    for ax in (ax_clr, ax_off):
        ax.axvspan(s_crit - 3, s_crit + 3, color='orange', alpha=0.15)
        ax.set_xlim(0, lap_len)
        ax.set_xlabel('track position s [m] (from the start line)')
        ax.grid(alpha=0.25)
    ax_clr.axhline(0, color='k', lw=1)
    ax_clr.set_ylim(-0.05, 0.6)
    ax_clr.set_ylabel('clearance to nearest cone [m]')
    ax_clr.set_title('Clearance along the lap (one line per lap)')
    ax_off.axhline(0, color='k', lw=1)
    ax_off.set_ylabel('lateral offset [m]  (+ blue side, - yellow side)')
    ax_off.set_title('Position in the lane along the lap')

    for i, r in enumerate(runs):
        d = r['data']
        ax_drift.plot(d['dist'], d['drift_xy'], lw=1, color=RUN_COLORS[i % len(RUN_COLORS)],
                      label=r['name'])
    ax_drift.set_xlabel('distance driven [m]')
    ax_drift.set_ylabel('odometry error vs truth [m]')
    ax_drift.set_title('Odometry drift')
    ax_drift.grid(alpha=0.25)
    ax_drift.legend(fontsize=7)

    def fmt(ms):
        return '—' if not ms else f"{ms['mean']:.2f} ± {ms['sd']:.2f} s (n={ms['n']})"
    lines = [
        f"Runs: {len(runs)}   completed {summary['laps_target']} laps: "
        f"{summary['runs_completed']}/{len(runs)}",
        f"Laps completed: {summary['laps_completed']}/{len(runs) * summary['laps_target']}",
        f"Contacts: {len(contacts)}",
    ] + [f"   {c['run']}  t={c['t']:.1f} s  {c['detail'].split(' ')[0]}" for c in contacts] + [
        f"Lap 1 (standing start): {fmt(summary['lap1_time_s'])}",
        f"Laps 2+ (flying):       {fmt(summary['flying_lap_time_s'])}",
        f"Odometry drift at the end: {drift_final and min(drift_final):.2f} – "
        f"{max(drift_final):.2f} m",
        f"Critical zone: s ≈ {s_crit:.0f} m, ({crit_x:.1f}, {crit_y:.1f})",
        f"   lowest clearance of {len(in_crit)}/{len(lows)} laps is here",
        f"Reference line: {ref_name}",
    ]
    ax_tab.axis('off')
    ax_tab.text(0.0, 1.0, '\n'.join(lines), va='top', family='monospace', fontsize=9.5)
    ax_tab.set_title('Summary', loc='left')

    png = os.path.join(out, 'batch_analysis.png')
    fig.savefig(png, dpi=110, bbox_inches='tight')

    with open(os.path.join(out, 'batch_summary.md'), 'w', encoding='utf-8') as f:
        f.write('| Run | Lap | Lowest clearance [m] | at (x, y) | s [m] | speed [m/s] '
                '| lateral offset [m] | contact |\n|---|---|---|---|---|---|---|---|\n')
        for lo in lows:
            f.write(f"| {lo['run']} | {lo['lap']} | {lo['clearance']:.3f} "
                    f"| ({lo['x']}, {lo['y']}) | {lo['s']} | {lo['speed']} | {lo['offset']} "
                    f"| {'yes' if lo['contact'] else ''} |\n")
        f.write('\n' + '\n'.join('    ' + ln for ln in lines) + '\n')
    print(f'[batch] {png}')
    print('\n'.join(lines))
    return 0


if __name__ == '__main__':
    sys.exit(main())
