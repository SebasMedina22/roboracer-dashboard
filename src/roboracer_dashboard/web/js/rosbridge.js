// Minimal rosbridge client (no libraries).
//
// rosbridge speaks plain JSON over a WebSocket:
//   browser -> {"op":"subscribe", "topic":"/scan", "type":"sensor_msgs/msg/LaserScan", ...}
//   rosbridge -> {"op":"publish", "topic":"/scan", "msg":{...the ROS message as JSON...}}
// That's all a dashboard needs.

export class RosBridge {
  constructor(url) {
    this.url = url;
    this.subs = new Map();          // topic -> {type, throttleMs, callbacks[], count, hz}
    this.onStatus = () => {};       // called with true/false when (dis)connected
    this._connect();
    setInterval(() => this._updateRates(), 1000);
  }

  // Register a callback for a topic. throttleMs asks rosbridge to send at most
  // one message every throttleMs (protects the browser from big/fast topics).
  subscribe(topic, type, callback, throttleMs = 0) {
    let sub = this.subs.get(topic);
    if (!sub) {
      sub = { type, throttleMs, callbacks: [], count: 0, hz: 0 };
      this.subs.set(topic, sub);
      if (this.connected) this._sendSubscribe(topic, sub);
    }
    sub.callbacks.push(callback);
  }

  // Messages per second actually received for a topic (after throttling).
  rate(topic) {
    const sub = this.subs.get(topic);
    return sub ? sub.hz : 0;
  }

  _connect() {
    this.ws = new WebSocket(this.url);

    this.ws.onopen = () => {
      this.connected = true;
      this.onStatus(true);
      for (const [topic, sub] of this.subs) this._sendSubscribe(topic, sub);
    };

    this.ws.onclose = () => {
      this.connected = false;
      this.onStatus(false);
      setTimeout(() => this._connect(), 2000);   // keep retrying
    };

    this.ws.onmessage = (event) => {
      let data;
      try {
        // LaserScan ranges can contain inf/NaN, which are not valid JSON.
        // Turn them into null before parsing.
        data = JSON.parse(event.data.replace(/\bNaN\b|-?\bInfinity\b/g, 'null'));
      } catch (err) {
        console.warn('Could not parse rosbridge message', err);
        return;
      }
      if (data.op !== 'publish') return;
      const sub = this.subs.get(data.topic);
      if (!sub) return;
      sub.count += 1;
      for (const cb of sub.callbacks) {
        try { cb(data.msg); } catch (err) { console.error(`Error handling ${data.topic}`, err); }
      }
    };
  }

  _sendSubscribe(topic, sub) {
    this.ws.send(JSON.stringify({
      op: 'subscribe',
      topic,
      type: sub.type,
      throttle_rate: sub.throttleMs,
      queue_length: 1,               // only the newest message, never a backlog
    }));
  }

  _updateRates() {
    for (const sub of this.subs.values()) {
      sub.hz = sub.count;
      sub.count = 0;
    }
  }
}
