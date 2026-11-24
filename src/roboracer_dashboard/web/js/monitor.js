// Run monitor panel: live state of an evaluation run (chocapinto_eval race_monitor).
// The monitor publishes a JSON string on /eval/status once per real second.
// Without it (e.g. on the real car) the panel just says so.

const STALE_MS = 5000;

function fmt(v, digits = 2, unit = '') {
  return v === null || v === undefined || Number.isNaN(v) ? '—' : `${Number(v).toFixed(digits)}${unit}`;
}

function clearanceClass(c) {
  if (c === null || c === undefined) return '';
  if (c < 0) return 'bad';
  if (c < 0.1) return 'bad';
  if (c < 0.2) return 'warn';
  return 'ok';
}

export function createMonitorPanel(el) {
  // el: {state, run, body, idle, simt, rtf, lap, dist, speed, clear, minclear, mode,
  //      contacts, drift, laps}
  let last = null;
  let lastAt = 0;

  function setText(node, text, cls) {
    node.textContent = text;
    node.className = cls || '';
  }

  function render() {
    const fresh = last && (Date.now() - lastAt < STALE_MS || last.state === 'done');
    el.body.hidden = !fresh;
    el.idle.hidden = !!fresh;
    if (!fresh) {
      setText(el.state, last ? 'no updates' : 'not running', 'pill muted');
      return;
    }
    const s = last;
    const stateText = s.state === 'done' ? `done · ${s.end_reason}` : s.state;
    const stateCls = s.state === 'done'
      ? (s.end_reason === 'laps_completed' ? 'pill ok' : 'pill bad') : 'pill ok';
    setText(el.state, stateText, stateCls);
    el.run.textContent = s.run || '';
    el.simt.textContent = fmt(s.sim_t, 0, ' s');
    el.rtf.textContent = s.rtf === null ? 'RTF —' : `RTF ${fmt(s.rtf, 2)}`;
    el.lap.textContent = `${Math.max(s.lap, s.laps_completed)} / ${s.laps_target}`;
    el.dist.textContent = fmt(s.distance, 0, ' m');
    el.speed.textContent = fmt(s.speed, 2, ' m/s');
    setText(el.clear, fmt(s.clearance, 2, ' m'), clearanceClass(s.clearance));
    setText(el.minclear, `min ${fmt(s.min_clearance, 3, ' m')}`, clearanceClass(s.min_clearance));
    el.mode.textContent = (s.mode || '—').toUpperCase();
    setText(el.contacts, String(s.contacts), s.contacts > 0 ? 'bad' : 'ok');
    el.drift.textContent = s.last_contact ? s.last_contact : `odom drift ${fmt(s.drift_xy, 2, ' m')}`;

    el.laps.replaceChildren();
    for (const lap of s.laps || []) {
      const tr = document.createElement('tr');
      for (const [v, cls] of [[lap.lap, ''], [fmt(lap.time, 1, ' s'), ''],
        [fmt(lap.avg_speed, 2, ' m/s'), ''],
        [fmt(lap.min_clearance, 3, ' m'), clearanceClass(lap.min_clearance)],
        [lap.contacts, lap.contacts > 0 ? 'bad' : '']]) {
        const td = document.createElement('td');
        td.textContent = v;
        if (cls) td.className = cls;
        tr.appendChild(td);
      }
      el.laps.appendChild(tr);
    }
  }

  setInterval(render, 1000);
  render();

  return {
    update(msg) {
      try {
        last = JSON.parse(msg.data);
        lastAt = Date.now();
        render();
      } catch (err) {
        console.warn('bad /eval/status message', err);
      }
    },
  };
}
