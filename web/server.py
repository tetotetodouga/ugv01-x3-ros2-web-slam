#!/usr/bin/env python3
"""HTTP dashboard and serialized software-stack supervisor for ROS 2 Jazzy."""
import json
import mimetypes
import os
from pathlib import Path
import shlex
import signal
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

WEB_DIR = Path(__file__).resolve().parent
STATIC_DIR = WEB_DIR / 'static'


def load_env(path):
    if path.is_file():
        for raw in path.read_text(encoding='utf-8').splitlines():
            line = raw.strip()
            if line and not line.startswith('#') and '=' in line:
                key, value = line.split('=', 1)
                os.environ.setdefault(key.strip(), value.strip().strip('"').strip("'"))


# The same user configuration is used by installed launch files and source runs.
load_env(Path.home() / '.config/ugv01/dashboard.env')
load_env(WEB_DIR / '.env')
WEB_HOST = os.getenv('WEB_HOST', '0.0.0.0')
WEB_PORT = int(os.getenv('WEB_PORT', '8080'))
DASHBOARD_TOKEN = os.getenv('DASHBOARD_TOKEN', '')
AUTO_CMD = os.getenv('AUTO_CMD',
    'ros2 run ugv01_room_explore auto_explore --ros-args '
    '-p cmd_topic:=/cmd_vel_auto -p scan_angle_offset_deg:=0.0 '
    '-p forward_wall:=0.045 -p forward_explore:=0.065 -p turn_speed:=0.28 '
    '-p corner_turn_speed:=0.18 -p back_speed:=0.045 '
    '-p front_emergency:=0.30 -p front_block:=0.55 -p front_clear:=0.75')
RUNTIME = SUPERVISOR = None


def status():
    nav = RUNTIME.node.read()
    return {'ok': True, 'mode': nav.get('drive_mode', 'JOY'),
            'navigation': nav, **SUPERVISOR.status()}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        print('[server]', fmt % args, flush=True)

    def send_json(self, code, payload):
        data = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def authorized(self):
        return not DASHBOARD_TOKEN or self.headers.get('X-Dashboard-Token', '') == DASHBOARD_TOKEN

    def do_GET(self):
        path = urlparse(self.path).path
        if path.startswith('/api/'):
            if not self.authorized():
                self.send_json(401, {'ok': False, 'error': 'unauthorized'})
            elif path == '/api/status':
                self.send_json(200, status())
            elif path == '/api/maps':
                self.send_json(200, {'ok': True, 'maps': SUPERVISOR.maps(), 'items': SUPERVISOR.map_items()})
            elif path == '/api/maps/preview':
                try:
                    name = parse_qs(urlparse(self.path).query).get('name', [''])[0]
                    data = SUPERVISOR.map_preview(name)
                    self.send_response(200)
                    self.send_header('Content-Type', 'image/png')
                    self.send_header('Content-Length', str(len(data)))
                    self.send_header('Cache-Control', 'private, max-age=300')
                    self.end_headers()
                    self.wfile.write(data)
                except (ValueError, KeyError, TypeError, OSError) as exc:
                    self.send_json(404, {'ok': False, 'error': str(exc)})
            else:
                self.send_json(404, {'ok': False, 'error': 'unknown endpoint'})
            return
        if path == '/':
            path = '/index.html'
        file_path = (STATIC_DIR / path.lstrip('/')).resolve()
        if not file_path.is_relative_to(STATIC_DIR.resolve()):
            self.send_error(403)
            return
        if not file_path.is_file():
            self.send_error(404)
            return
        data = file_path.read_bytes()
        self.send_response(200)
        self.send_header('Content-Type', mimetypes.guess_type(str(file_path))[0] or 'application/octet-stream')
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        self.wfile.write(data)

    def do_POST(self):
        if not self.authorized():
            self.send_json(401, {'ok': False, 'error': 'unauthorized'})
            return
        path = urlparse(self.path).path
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 <= length <= 8192:
                raise ValueError('Request too large')
            body = json.loads(self.rfile.read(length) or b'{}')
            if not isinstance(body, dict):
                raise ValueError('Expected JSON object')
            result = None
            if path == '/api/mode/auto':
                SUPERVISOR.drive('AUTO')
            elif path == '/api/mode/joy':
                SUPERVISOR.drive('JOY')
            elif path in ('/api/stop', '/api/stop_robot', '/api/navigation/cancel'):
                SUPERVISOR.stop()
            elif path == '/api/navigation/mode':
                SUPERVISOR.switch(body.get('mode'), body.get('map'))
            elif path == '/api/navigation/goal':
                with SUPERVISOR.lock:
                    SUPERVISOR.stop_auto()
                    result = RUNTIME.node.invoke('goal', **{k: body[k] for k in ('x', 'y', 'yaw')})
            elif path == '/api/localization/initialpose':
                SUPERVISOR.stop()
                result = RUNTIME.node.invoke('initialpose', **{k: body[k] for k in ('x', 'y', 'yaw')})
            elif path == '/api/localization/global':
                SUPERVISOR.stop()
                result = RUNTIME.node.invoke('global')
            elif path == '/api/maps/save':
                result = SUPERVISOR.save_map(body.get('name'))
            else:
                self.send_json(404, {'ok': False, 'error': 'unknown endpoint'})
                return
            payload = result if result is not None else status()
            self.send_json(200 if payload.get('ok') else 409, payload)
        except (ValueError, KeyError, TypeError) as exc:
            self.send_json(400, {'ok': False, 'error': str(exc)})
        except Exception as exc:
            self.send_json(500, {'ok': False, 'error': str(exc)})


def main():
    global RUNTIME, SUPERVISOR
    try:
        from ugv01_room_explore.navigation_bridge import RosRuntime
        from ugv01_room_explore.stack_supervisor import StackSupervisor
    except ImportError:
        if os.getenv('UGV_ROS_SOURCED'):
            raise
        ros_setup = os.getenv('ROS_SETUP', '/opt/ros/jazzy/setup.bash')
        ws_setup = os.getenv('WORKSPACE_SETUP', str(Path.home() / 'ros2_ws/install/setup.bash'))
        command = (f'source {shlex.quote(ros_setup)} && source {shlex.quote(ws_setup)} && '
                   f'exec python3 {shlex.quote(str(Path(__file__).resolve()))}')
        os.environ['UGV_ROS_SOURCED'] = '1'
        os.execvp('bash', ['bash', '-c', command])
    # Fail a duplicate bind before creating any ROS publishers or sending STOP.
    server = ThreadingHTTPServer((WEB_HOST, WEB_PORT), Handler)
    server.daemon_threads = True
    RUNTIME = RosRuntime()
    SUPERVISOR = StackSupervisor(
        RUNTIME.node, os.getenv('UGV_MANAGED_STACK', '0') == '1',
        os.getenv('UGV_MAPS_DIR', str(Path.home() / '.ros/ugv01/maps')),
        os.getenv('UGV_LOGS_DIR', str(Path.home() / '.ros/ugv01/logs')), AUTO_CMD)
    def stopping(_signal, _frame):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, stopping)
    signal.signal(signal.SIGTERM, stopping)
    try:
        if SUPERVISOR.managed:
            SUPERVISOR.switch('mapping')
        threading.Thread(target=SUPERVISOR.monitor, daemon=True).start()
        print(f'UGV01 dashboard http://{WEB_HOST}:{WEB_PORT}; managed={SUPERVISOR.managed}', flush=True)
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        SUPERVISOR.close()
        if server:
            server.server_close()
        RUNTIME.close()


if __name__ == '__main__':
    main()
