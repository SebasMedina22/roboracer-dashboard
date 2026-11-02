"""Serve the dashboard web page (HTML/JS) over HTTP.

The page itself talks to ROS through rosbridge (WebSocket :9090) and gets the
camera from web_video_server (HTTP :8080). This node only hands the static
files to the browser, so you can open http://localhost:8000.
"""
import functools
import os
import threading
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node


class NoCacheHandler(SimpleHTTPRequestHandler):
    """Static file handler that disables browser caching (so edits show on refresh)."""

    extensions_map = {
        **SimpleHTTPRequestHandler.extensions_map,
        '.js': 'text/javascript',
        '.css': 'text/css',
        '.html': 'text/html',
    }

    def end_headers(self):
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()

    def log_message(self, fmt, *args):  # keep the terminal quiet
        pass


class DashboardWebServer(Node):

    def __init__(self):
        super().__init__('dashboard_web_server')
        port = int(self.declare_parameter('port', 8000).value)
        web_dir = os.path.join(get_package_share_directory('roboracer_dashboard'), 'web')

        handler = functools.partial(NoCacheHandler, directory=web_dir)
        self.httpd = ThreadingHTTPServer(('0.0.0.0', port), handler)
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

        self.get_logger().info(f'Dashboard: http://localhost:{port}   (files: {web_dir})')

    def stop(self):
        self.httpd.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = DashboardWebServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
