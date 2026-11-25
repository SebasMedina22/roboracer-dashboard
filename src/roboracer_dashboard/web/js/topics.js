// Automatic topic discovery, so the dashboard works on the real car without
// knowing its topic names in advance.
//
// rosapi (started by dashboard.launch.py) answers the service /rosapi/topics with
// every topic name and its message type. For each panel we know the TYPE we need
// (e.g. sensor_msgs/msg/Image for the camera), so:
//   1. the expected name exists          -> use it
//   2. it does not exist                 -> pick the best topic of the right type
//   3. nothing of that type (yet)        -> keep waiting for the expected name
// A topic given in the URL, or chosen in a panel's drop-down, is never replaced.

// score(name) > 0: usable candidate, higher = better match. 0: never pick.
export const PANELS = {
  camera: {
    type: 'sensor_msgs/msg/Image',
    score: (t) => {
      if (/depth|infra|ir\b/i.test(t)) return 0;          // web_video_server shows colour only
      return 1 + (/color|rgb/i.test(t) ? 2 : 0) + (/image_raw$/.test(t) ? 1 : 0);
    },
  },
  scan: { type: 'sensor_msgs/msg/LaserScan', score: (t) => (t === '/scan' ? 3 : /scan/.test(t) ? 2 : 1) },
  odom: { type: 'nav_msgs/msg/Odometry', score: (t) => (t === '/odom' ? 3 : /odom/.test(t) ? 2 : 1) },
  map: {
    type: 'nav_msgs/msg/OccupancyGrid',
    score: (t) => (/costmap/.test(t) ? 0 : t === '/map' ? 3 : /map/.test(t) ? 2 : 1),
  },
};

// Topics of `type` that the panel could show, best first.
export function candidates(list, type, score) {
  return list
    .filter((t) => t.type === type && score(t.name) > 0)
    .map((t) => t.name)
    .sort((a, b) => score(b) - score(a) || a.localeCompare(b));
}

// Which topic to use now (rules 1-3 above).
export function choose(current, pinned, list, panel) {
  const names = new Set(list.map((t) => t.name));
  if (pinned || names.has(current)) return current;
  const best = candidates(list, panel.type, panel.score)[0];
  return best || current;
}

// Ask rosapi for {name, type} of every topic. Resolves to null if rosapi is not running.
export async function listTopics(ros) {
  try {
    const res = await ros.callService('/rosapi/topics', 'rosapi_msgs/srv/Topics', {});
    return res.topics.map((name, i) => ({ name, type: res.types[i] }));
  } catch (err) {
    return null;
  }
}

// One drop-down per panel header. onChange(newTopic) is called when the topic changes,
// automatically or by hand.
export class TopicSelector {
  constructor(key, select, initial, pinned, onChange) {
    this.key = key;
    this.panel = PANELS[key];
    this.select = select;
    this.current = initial;
    this.pinned = pinned;         // from the URL or chosen by hand: never auto-replaced
    this.auto = false;            // true when the current topic was picked automatically
    this.onChange = onChange;
    this.options = [];
    this._render();
    select.addEventListener('change', () => {
      this.pinned = true;
      this._set(select.value, false);
    });
  }

  // Called with a fresh topic list (every few seconds).
  update(list) {
    const next = choose(this.current, this.pinned, list, this.panel);
    if (next !== this.current) this._set(next, true);
    const opts = candidates(list, this.panel.type, this.panel.score);
    if (opts.join() !== this.options.join()) {
      this.options = opts;
      this._render();
    }
  }

  _set(topic, auto) {
    this.current = topic;
    this.auto = auto;
    this._render();
    this.onChange(topic);
  }

  _render() {
    const names = this.options.includes(this.current) ? this.options : [this.current, ...this.options];
    this.select.replaceChildren(...names.map((n) => {
      const o = document.createElement('option');
      o.value = n;
      o.textContent = n;
      return o;
    }));
    this.select.value = this.current;
    this.select.classList.toggle('auto', this.auto);
    this.select.title = this.auto
      ? `auto-detected (expected topic not found). Other ${this.panel.type} topics in the list.`
      : `${this.panel.type} topics found in ROS`;
  }
}
