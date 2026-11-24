#!/usr/bin/env python3
"""Plot one evaluated run: track, trajectory coloured by speed, contacts, modes, summary.

Usage: python3 scripts/plot_run.py <run_dir>      -> <run_dir>/run.png
Needs matplotlib (sudo apt install -y python3-matplotlib).
"""
import csv
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src', 'chocapinto_eval'))   # works without sourcing

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
except ImportError:
    sys.exit('matplotlib missing: sudo apt install -y python3-matplotlib')

from chocapinto_eval.metrics import load_cones_from_world  # noqa: E402

MODE_STYLE = {'hold': ('#f39c12', 'v'), 'explore': ('#c0392b', 'v'), 'stop': ('#000000', 's'),
              'wait': ('#7f8c8d', 'o')}


def read_csv(path):
    if not os.path.exists(path):
        return []
    with open(path, newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return math.nan


def main(run_dir: str) -> str:
    with open(os.path.join(run_dir, 'summary.json'), encoding='utf-8') as f:
        s = json.load(f)
    rows = read_csv(os.path.join(run_dir, 'trajectory.csv'))
    events = read_csv(os.path.join(run_dir, 'events.csv'))
    t = [num(r['t']) for r in rows]
    x = [num(r['x']) for r in rows]
    y = [num(r['y']) for r in rows]
    speed = [num(r['speed']) for r in rows]
    clear = [num(r['clearance']) for r in rows]
    ox = [num(r['odom_x']) for r in rows]
    oy = [num(r['odom_y']) for r in rows]

    fig = plt.figure(figsize=(16, 8))
    gs = fig.add_gridspec(3, 3, width_ratios=[2.2, 1, 1], height_ratios=[0.8, 1, 1])
    ax = fig.add_subplot(gs[:, 0])

    # track
    try:
        cones, colors, _ = load_cones_from_world(s['world_file'])
        for (cx, cy), c in zip(cones, colors):
            ax.add_patch(Circle((cx, cy), 0.117, color='#1f4fd1' if c == 'blue' else '#e0b000',
                                alpha=0.8, lw=0))
    except OSError:
        ax.text(0.02, 0.98, 'world file not found', transform=ax.transAxes, va='top')
    ax.plot([0, 0], [-2.5, 2.5], 'r--', lw=1)

    # trajectory (truth coloured by speed, odometry dashed)
    if rows:
        if not all(math.isnan(v) for v in ox):
            ax.plot(ox, oy, '--', color='gray', lw=0.8, label='según /odom')
        sc = ax.scatter(x, y, c=speed, s=3, cmap='viridis', vmin=0,
                        vmax=max(1.0, max(v for v in speed if not math.isnan(v))))
        fig.colorbar(sc, ax=ax, fraction=0.03, pad=0.01, label='velocidad real [m/s]')
        ax.plot(x[0], y[0], 'go', ms=8, label='salida')
    for e in events:
        ex, ey = num(e['x']), num(e['y'])
        if e['kind'] == 'contact':
            ax.plot(ex, ey, 'rx', ms=14, mew=3)
            ax.annotate(e['detail'].split(' ')[0], (ex, ey), xytext=(6, 6),
                        textcoords='offset points', color='r', fontsize=8)
        elif e['kind'] == 'mode' and e['detail'] in MODE_STYLE and not math.isnan(ex):
            col, mk = MODE_STYLE[e['detail']]
            ax.plot(ex, ey, mk, color=col, ms=7)
        elif e['kind'] == 'stuck':
            ax.plot(ex, ey, 'ks', ms=10, mfc='none', mew=2)
    for mode, (col, mk) in MODE_STYLE.items():
        if any(e['kind'] == 'mode' and e['detail'] == mode for e in events):
            ax.plot([], [], mk, color=col, label=f'modo {mode}')
    if any(e['kind'] == 'contact' for e in events):
        ax.plot([], [], 'rx', mew=3, label='choque')
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.legend(loc='lower left', fontsize=8)
    ax.set_title(os.path.basename(os.path.normpath(run_dir)))

    # summary text
    axt = fig.add_subplot(gs[0, 1:])
    axt.axis('off')
    laps = s.get('laps', [])
    lines = [
        f"Fin: {s['end_reason']}    vueltas: {s['laps_completed']}/{s['laps_target']}",
        f"Tiempo simulado: {s['sim_time']} s    real: {s.get('wall_time')} s    "
        f"RTF medio: {s.get('rtf_avg')}",
        f"Distancia: {s['distance']} m    velocidad media: {s['avg_speed']} m/s    "
        f"máx: {s['max_speed']} m/s",
        f"Choques: {s['contacts']}    margen mínimo a un cono: {s['min_clearance']} m",
        f"Deriva de /odom al final: {s['odom_drift_xy']} m, {s['odom_drift_yaw_deg']}°",
        '',
        'Vuelta | tiempo [s] | v media [m/s] | margen mín [m] | choques | deriva [m]',
    ]
    for lap in laps:
        lines.append(f"  {lap['lap']}    | {lap['time']:9.2f}  | {lap['avg_speed']:12.2f}  | "
                     f"{lap['min_clearance']:13.3f}  | {lap['contacts']:7d}  | {lap['drift_xy']}")
    axt.text(0, 1, '\n'.join(lines), va='top', family='monospace', fontsize=10)

    # time series
    lap_t = [num(e['t']) for e in events if e['kind'] == 'lap']
    hit_t = [num(e['t']) for e in events if e['kind'] == 'contact']
    for row, (vals, label) in enumerate([(speed, 'velocidad [m/s]'),
                                         (clear, 'margen a cono [m]')], start=1):
        a = fig.add_subplot(gs[row, 1:])
        a.plot(t, vals, lw=0.8)
        for lt in lap_t:
            a.axvline(lt, color='g', lw=0.8, ls='--')
        for ht in hit_t:
            a.axvline(ht, color='r', lw=1.2)
        if row == 2:
            a.axhline(0, color='r', lw=0.6)
            a.set_xlabel('tiempo simulado [s]  (verde = vuelta, rojo = choque)')
        a.set_ylabel(label)
        a.grid(alpha=0.3)

    fig.tight_layout()
    out = os.path.join(run_dir, 'run.png')
    fig.savefig(out, dpi=100)
    plt.close(fig)
    print(f'[eval] plot -> {out}')
    return out


if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
