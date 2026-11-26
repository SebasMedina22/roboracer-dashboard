"""Small MJPEG video server: a stand-in for web_video_server, in pure Python.

Started by dashboard.launch.py only when web_video_server is not installed (e.g. on the
car's Jetson, where we cannot install packages). It speaks the same URL the dashboard
already uses:

    http://<host>:8080/stream?topic=/camera/.../image_raw&type=mjpeg&quality=60

For every topic a browser asks for it subscribes to the sensor_msgs/Image topic,
shrinks each frame to at most `max_width` pixels wide, encodes it as JPEG and sends it
as a multipart MJPEG stream. Frames are only encoded while someone is watching, and at
most `max_fps` times per second, so an idle dashboard costs nothing.
"""
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import numpy as np

try:
    import cv2
except ImportError:                     # pragma: no cover - fallback if OpenCV is missing
    cv2 = None

BOUNDARY = 'frame'


# ---------------------------------------------------------------- image conversion (no ROS)
def image_to_bgr(msg):
    """sensor_msgs/Image -> HxWx3 uint8 BGR array (None for unsupported encodings)."""
    h, w, step, enc = msg.height, msg.width, msg.step, msg.encoding.lower()
    buf = np.frombuffer(bytes(msg.data) if not isinstance(msg.data, (bytes, bytearray))
                        else msg.data, dtype=np.uint8)
    if enc in ('rgb8', 'bgr8'):
        img = buf.reshape(h, step)[:, :w * 3].reshape(h, w, 3)
        return img[:, :, ::-1] if enc == 'rgb8' else img
    if enc in ('rgba8', 'bgra8'):
        img = buf.reshape(h, step)[:, :w * 4].reshape(h, w, 4)[:, :, :3]
        return img[:, :, ::-1] if enc == 'rgba8' else img
    if enc in ('mono8', '8uc1'):
        g = buf.reshape(h, step)[:, :w]
        return np.repeat(g[:, :, None], 3, axis=2)
    if enc in ('16uc1', 'mono16'):      # depth in mm: show 0..5 m as grey
        d = np.frombuffer(buf.tobytes(), dtype='<u2').reshape(h, step // 2)[:, :w]
        g = np.clip(d / 5000.0 * 255.0, 0, 255).astype(np.uint8)
        return np.repeat(g[:, :, None], 3, axis=2)
    return None


def encode_jpeg(bgr, max_width, quality):
    """Downscale (keeping the aspect ratio) and JPEG-encode a BGR image."""
    h, w = bgr.shape[:2]
    if w > max_width:
        new = (max_width, max(1, int(round(h * max_width / w))))
        if cv2 is not None:
            bgr = cv2.resize(bgr, new, interpolation=cv2.INTER_AREA)
        else:
            step = int(np.ceil(w / max_width))
            bgr = bgr[::step, ::step]
    if cv2 is not None:
        ok, jpg = cv2.imencode('.jpg', bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
        return jpg.tobytes() if ok else None
    from io import BytesIO                 # pragma: no cover - Pillow fallback
    from PIL import Image
    out = BytesIO()
    Image.fromarray(bgr[:, :, ::-1]).save(out, format='JPEG', quality=int(quality))
    return out.getvalue()


# ---------------------------------------------------------------- frames shared with HTTP
class FrameHub:
    """Latest JPEG per topic, with a condition so streams wake up on new frames."""

    def __init__(self):
        self.cond = threading.Condition()
        self.frames = {}        # topic -> (seq, jpeg bytes)
        self.watchers = {}      # topic -> number of open streams
        self.quality = {}       # topic -> JPEG quality asked by the newest client

    def watch(self, topic, quality, delta):
        with self.cond:
            self.watchers[topic] = max(0, self.watchers.get(topic, 0) + delta)
            if delta > 0:
                self.quality[topic] = quality

    def watched(self, topic):
        with self.cond:
            return self.watchers.get(topic, 0) > 0

    def put(self, topic, jpeg):
        with self.cond:
            seq = self.frames.get(topic, (0, b''))[0] + 1
            self.frames[topic] = (seq, jpeg)
            self.cond.notify_all()

    def wait(self, topic, last_seq, timeout):
        with self.cond:
            self.cond.wait_for(lambda: self.frames.get(topic, (0, b''))[0] != last_seq,
                               timeout=timeout)
            return self.frames.get(topic, (last_seq, None))


def make_handler(hub, on_topic):
    """HTTP handler class bound to a FrameHub; on_topic(topic) makes sure we subscribe."""

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.0'

        def log_message(self, fmt, *args):
            pass

        def do_GET(self):
            url = urlparse(self.path)
            q = parse_qs(url.query)
            topic = (q.get('topic') or [''])[0]
            if url.path not in ('/stream', '/snapshot') or not topic.startswith('/'):
                self.send_error(404, 'use /stream?topic=/some/image_raw')
                return
            quality = int((q.get('quality') or ['60'])[0])
            on_topic(topic)
            if url.path == '/snapshot':
                seq, jpg = hub.wait(topic, 0, timeout=5.0)
                if not jpg:
                    hub.watch(topic, quality, +1)
                    seq, jpg = hub.wait(topic, 0, timeout=5.0)
                    hub.watch(topic, quality, -1)
                if not jpg:
                    self.send_error(503, f'no image on {topic} yet')
                    return
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(jpg)))
                self.end_headers()
                self.wfile.write(jpg)
                return
            self.send_response(200)
            self.send_header('Content-Type', f'multipart/x-mixed-replace; boundary={BOUNDARY}')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            hub.watch(topic, quality, +1)
            seq = 0
            try:
                while True:
                    new_seq, jpg = hub.wait(topic, seq, timeout=5.0)
                    if jpg is None or new_seq == seq:
                        continue                      # no new frame yet: keep waiting
                    seq = new_seq
                    self.wfile.write(f'--{BOUNDARY}\r\nContent-Type: image/jpeg\r\n'
                                     f'Content-Length: {len(jpg)}\r\n\r\n'.encode())
                    self.wfile.write(jpg)
                    self.wfile.write(b'\r\n')
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass                                    # browser closed the stream
            finally:
                hub.watch(topic, quality, -1)

    return Handler


# ---------------------------------------------------------------- ROS node
def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    rclpy.init(args=args)
    node = Node('video_server')
    port = int(node.declare_parameter('port', 8080).value)
    max_width = int(node.declare_parameter('max_width', 640).value)
    max_fps = float(node.declare_parameter('max_fps', 10.0).value)
    hub = FrameHub()
    subs, last_encode, lock = {}, {}, threading.Lock()

    def on_image(topic, msg):
        if not hub.watched(topic):
            return                                      # nobody watching: no work
        now = time.monotonic()
        if now - last_encode.get(topic, 0.0) < 1.0 / max_fps:
            return
        last_encode[topic] = now
        bgr = image_to_bgr(msg)
        if bgr is None:
            node.get_logger().warn(f'{topic}: encoding {msg.encoding} not supported',
                                   throttle_duration_sec=10.0)
            return
        jpg = encode_jpeg(bgr, max_width, hub.quality.get(topic, 60))
        if jpg:
            hub.put(topic, jpg)

    def ensure_subscribed(topic):
        with lock:
            if topic not in subs:
                subs[topic] = node.create_subscription(
                    Image, topic, lambda m, t=topic: on_image(t, m), qos_profile_sensor_data)
                node.get_logger().info(f'streaming {topic}')

    httpd = ThreadingHTTPServer(('0.0.0.0', port), make_handler(hub, ensure_subscribed))
    httpd.daemon_threads = True
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    node.get_logger().info(f'video_server (Python stand-in for web_video_server) on :{port}, '
                           f'max {max_width} px wide, {max_fps:g} fps, encoder '
                           f'{"OpenCV" if cv2 is not None else "Pillow"}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        httpd.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
