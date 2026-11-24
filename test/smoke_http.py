#!/usr/bin/env python3
"""Exercise HTTP mode switching without hardware, in a separate ROS domain."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time
import urllib.error
import urllib.request

if os.environ.get('ROS_DOMAIN_ID') != '92' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise SystemExit('Use ROS_DOMAIN_ID=92 ROS_LOCALHOST_ONLY=1')


def request(path, body=None, token='software-smoke'):
    req = urllib.request.Request('http://127.0.0.1:18080' + path,
        data=json.dumps(body).encode() if body is not None else None,
        headers={'Content-Type': 'application/json', 'X-Dashboard-Token': token})
    try:
        with urllib.request.urlopen(req, timeout=25) as response:
            return response.status, json.load(response)
    except urllib.error.HTTPError as exc:
        return exc.code, json.load(exc)


def main():
    root = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix='ugv01-http-smoke-') as folder:
        env = dict(os.environ, WEB_HOST='127.0.0.1', WEB_PORT='18080',
                   DASHBOARD_TOKEN='software-smoke', UGV_MANAGED_STACK='1',
                   UGV_MAPS_DIR=folder + '/maps', UGV_LOGS_DIR=folder + '/logs')
        log_path = Path('/tmp/ugv01-http-smoke.log')
        with log_path.open('w') as log:
            process = subprocess.Popen(['python3', '-u', str(root / 'web/server.py')],
                env=env, stdout=log, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic() + 20
                while True:
                    try:
                        code, status = request('/api/status')
                        if code == 200 and status.get('slam_pid'):
                            break
                    except (OSError, ValueError):
                        pass
                    if time.monotonic() > deadline:
                        raise AssertionError('HTTP startup timeout')
                    time.sleep(.2)
                assert status['mode'] == 'JOY'
                original_slam_pid = status['slam_pid']
                assert request('/api/status', token='wrong')[0] == 401
                assert request('/api/navigation/goal', {'x': 1, 'y': 0, 'yaw': 0})[0] == 400
                code, status = request('/api/navigation/mode', {'mode': 'nav_slam'})
                assert code == 200, status
                assert status['slam_pid'] == original_slam_pid
                assert status['stack_pid'] is not None
                assert request('/api/navigation/mode', {'mode': 'nav_map', 'map': '../bad'})[0] == 400
                code, status = request('/api/navigation/mode', {'mode': 'mapping'})
                assert code == 200, status
                assert status['slam_pid'] == original_slam_pid
                assert status['stack_pid'] is None
                assert request('/api/maps/save', {'name': '../bad'})[0] == 400
                assert request('/api/stop', {})[0] == 200
                assert request('/api/maps')[1]['maps'] == []
                print('HTTP SMOKE PASSED: auth, validation, goal guard, SLAM preservation, mode switch, STOP', flush=True)
            except Exception:
                print(log_path.read_text()[-12000:], flush=True)
                raise
            finally:
                process.terminate()
                process.wait(timeout=25)


if __name__ == '__main__':
    main()
