// Logs panel: messages from every node (/rosout, rcl_interfaces/msg/Log).
// Repeated messages are grouped ("x120") so one noisy node cannot flood the list.

const LEVELS = { 10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL' };
const MAX_ENTRIES = 300;

function levelName(level) {
  return LEVELS[level] || String(level);
}

function stampText(stamp) {
  if (!stamp) return '';
  const sec = stamp.sec + (stamp.nanosec || 0) * 1e-9;
  if (sec < 1e8) return `t=${sec.toFixed(1)}s`;            // simulation time
  return new Date(sec * 1000).toLocaleTimeString();        // wall clock (real car)
}

export function createLogsPanel(listEl, buttons, countEl) {
  // buttons: {all, warn, monitor, clear}
  const entries = [];          // newest first: {key, name, level, msg, count, stamp}
  const byKey = new Map();
  let filter = 'all';
  let dirty = false;
  let total = 0;

  function visible(e) {
    if (filter === 'warn') return e.level >= 30;
    if (filter === 'monitor') return e.name.endsWith('race_monitor') || e.name.endsWith('launch');
    return true;
  }

  function render() {
    dirty = false;
    const frag = document.createDocumentFragment();
    let shown = 0;
    for (const e of entries) {
      if (!visible(e)) continue;
      const row = document.createElement('div');
      row.className = `log-row lvl-${levelName(e.level).toLowerCase()}`;
      const time = document.createElement('span');
      time.className = 'log-time';
      time.textContent = stampText(e.stamp);
      const name = document.createElement('span');
      name.className = 'log-name';
      name.textContent = e.name;
      const text = document.createElement('span');
      text.className = 'log-msg';
      text.textContent = e.msg;
      row.append(time, name, text);
      if (e.count > 1) {
        const count = document.createElement('span');
        count.className = 'log-count';
        count.textContent = `×${e.count}`;
        row.appendChild(count);
      }
      frag.appendChild(row);
      shown += 1;
    }
    listEl.replaceChildren(frag);
    countEl.textContent = `${shown} shown · ${total} received`;
  }

  function schedule() {
    if (!dirty) {
      dirty = true;
      requestAnimationFrame(render);
    }
  }

  function setFilter(name) {
    filter = name;
    for (const key of ['all', 'warn', 'monitor']) buttons[key].classList.toggle('active', key === name);
    schedule();
  }
  buttons.all.onclick = () => setFilter('all');
  buttons.warn.onclick = () => setFilter('warn');
  buttons.monitor.onclick = () => setFilter('monitor');
  buttons.clear.onclick = () => { entries.length = 0; byKey.clear(); total = 0; schedule(); };
  setFilter('all');

  return {
    add(msg) {
      total += 1;
      const key = `${msg.name}|${msg.level}|${msg.msg}`;
      let e = byKey.get(key);
      if (e) {
        e.count += 1;
        e.stamp = msg.stamp;
        entries.splice(entries.indexOf(e), 1);       // move to the top
      } else {
        e = { key, name: msg.name, level: msg.level, msg: msg.msg, count: 1, stamp: msg.stamp };
        byKey.set(key, e);
        if (entries.length >= MAX_ENTRIES) byKey.delete(entries.pop().key);
      }
      entries.unshift(e);
      schedule();
    },
  };
}
