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
    this.pending = new Map();       // service call id -> {resolve, reject, timer}
    this.nextId = 1;
    this.onStatus = () => {};       // called with true/false when (dis)connected
    this._connect();
    setInterval(() => this._updateRates(), 1000);
  }

  // Register a callback for a topic. throttleMs asks rosbridge to send at most
  // one message every throttleMs (protects the browser from big/fast topics).
  // queueLength 1 = only the newest message; use more for topics where every message
  // matters (logs).
  subscribe(topic, type, callback, throttleMs = 0, queueLength = 1) {
    let sub = this.subs.get(topic);
    if (!sub) {
      sub = { type, throttleMs, queueLength, callbacks: [], count: 0, hz: 0 };
      this.subs.set(topic, sub);
      if (this.connected) this._sendSubscribe(topic, sub);
    }
    sub.callbacks.push(callback);
  }

  // Stop receiving a topic (used when a panel switches to another topic).
  unsubscribe(topic) {
    if (!this.subs.delete(topic)) return;
    if (this.connected) this.ws.send(JSON.stringify({ op: 'unsubscribe', topic }));
  }

  // Call a ROS service; resolves with the response values, rejects after timeoutMs
  // or if the service fails (e.g. rosapi not running).
  callService(service, type, args = {}, timeoutMs = 3000) {
    if (!this.connected) return Promise.reject(new Error('not connected'));
    const id = `call_${this.nextId++}`;
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        this.pending.delete(id);
        reject(new Error(`${service}: timeout`));
      }, timeoutMs);
      this.pending.set(id, { resolve, reject, timer });
      this.ws.send(JSON.stringify({ op: 'call_service', id, service, type, args }));
    });
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
      if (data.op === 'service_response') {
        const call = this.pending.get(data.id);
        if (!call) return;
        this.pending.delete(data.id);
        clearTimeout(call.timer);
        if (data.result === false) call.reject(new Error(String(data.values)));
        else call.resolve(data.values);
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
      queue_length: sub.queueLength, // 1 = only the newest message, never a backlog
    }));
  }

  _updateRates() {
    for (const sub of this.subs.values()) {
      sub.hz = sub.count;
      sub.count = 0;
    }
  }
}
