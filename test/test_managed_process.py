"""Abrupt parent death must not leave a managed software child alive."""
import os
from pathlib import Path
import subprocess
import sys
import threading
import time

import pytest

from ugv01_room_explore.stack_supervisor import StackSupervisor, stop_process


@pytest.mark.skipif(sys.platform != 'linux', reason='Linux parent-death signal')
def test_request_thread_exit_does_not_kill_managed_child(tmp_path):
    class Bridge:
        def invoke(self, *args, **kwargs):
            pass

    supervisor = StackSupervisor(Bridge(), False, tmp_path / 'maps', tmp_path / 'logs', '')
    children = []
    worker = threading.Thread(target=lambda: children.append(supervisor.spawn(
        [sys.executable, '-u', '-c', 'import time; print("READY", flush=True); time.sleep(60)'],
        'child.log')))
    try:
        worker.start()
        worker.join(timeout=3)
        assert not worker.is_alive()
        assert children
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            if 'READY' in (tmp_path / 'logs/child.log').read_text():
                break
            time.sleep(.05)
        else:
            pytest.fail('Managed child did not start')
        time.sleep(.2)
        assert children[0].poll() is None, 'HTTP request thread exit killed its software child'
    finally:
        for child in children:
            stop_process(child)
        supervisor.close()


@pytest.mark.skipif(sys.platform != 'linux', reason='Linux parent-death signal')
def test_managed_child_exits_when_parent_is_killed():
    code = '''
import os, subprocess, sys, time
child = subprocess.Popen([sys.executable, '-m', 'ugv01_room_explore.managed_process',
    str(os.getpid()), sys.executable, '-u', '-c',
    'import time; print("READY", flush=True); time.sleep(60)'],
    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, start_new_session=True)
ready = child.stdout.readline().strip()
print(child.pid, ready, flush=True)
time.sleep(60)
'''
    parent = subprocess.Popen([sys.executable, '-u', '-c', code], stdout=subprocess.PIPE, text=True)
    child_pid = None
    try:
        pid, ready = parent.stdout.readline().strip().split()
        child_pid = int(pid)
        assert ready == 'READY'
        parent.kill()
        parent.wait(timeout=3)
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            try:
                state = (Path('/proc') / pid / 'stat').read_text().rsplit(') ', 1)[1].split()[0]
                if state in ('Z', 'X'):
                    return
            except FileNotFoundError:
                return
            time.sleep(.05)
        pytest.fail('Managed child survived dashboard parent death')
    finally:
        if parent.poll() is None:
            parent.kill()
            parent.wait(timeout=3)
        if child_pid:
            try:
                os.killpg(child_pid, 9)
            except ProcessLookupError:
                pass
