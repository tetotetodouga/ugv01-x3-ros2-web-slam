"""Own only software processes; never start motor drivers from an HTTP request."""
from concurrent.futures import ThreadPoolExecutor
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

from ugv01_room_explore.map_gallery import list_map_cards, map_name, map_preview


def stop_process(process):
    if process is None:
        return
    for sig, wait in ((signal.SIGINT, 7), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
        try:
            if sig == signal.SIGINT and process.poll() is None:
                # ros2 launch forwards SIGINT to its children itself. Sending
                # it to the whole group first interrupts ROS teardown twice,
                # including nodes still installing their signal handlers.
                os.kill(process.pid, sig)
            else:
                os.killpg(process.pid, sig)
        except ProcessLookupError:
            process.poll()
            return
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline:
            process.poll()  # reap the launch parent, then also wait for its children
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                return
            time.sleep(0.05)
        # Escalate even if the launch parent already exited: a detached software
        # stack must not leave a second SLAM TF publisher running in this group.


class StackSupervisor:
    def __init__(self, bridge, managed, maps_dir, logs_dir, auto_command):
        self.bridge, self.managed = bridge, managed
        self.maps_dir = Path(maps_dir).expanduser().resolve()
        self.logs_dir = Path(logs_dir).expanduser().resolve()
        self.maps_dir.mkdir(parents=True, exist_ok=True)
        self.logs_dir.mkdir(parents=True, exist_ok=True)
        self.auto_command = auto_command
        self.lock = threading.RLock()
        self.process = self.auto_process = self.slam_process = None
        self.mode = 'mapping'
        self.selected_map = None
        self.error = ''
        self.closed = threading.Event()
        # Linux parent-death signals follow the thread that creates a child.
        # HTTP request threads are short-lived, so launch from one persistent
        # worker and keep it alive until all software children have stopped.
        self.launcher = ThreadPoolExecutor(max_workers=1, thread_name_prefix='ugv01-launch')

    def spawn(self, command, log_name):
        with (self.logs_dir / log_name).open('a', buffering=1) as log:
            command = [sys.executable, '-m', 'ugv01_room_explore.managed_process',
                       str(os.getpid()), *command]
            return self.launcher.submit(subprocess.Popen, command, stdout=log,
                stderr=subprocess.STDOUT, start_new_session=True).result()

    def maps(self):
        return sorted(p.stem for p in self.maps_dir.glob('*.yaml') if p.is_file())

    def map_items(self):
        return list_map_cards(self.maps_dir)

    def map_preview(self, name):
        return map_preview(self.maps_dir, name)

    def switch(self, mode, selected_map=None):
        if not self.managed:
            raise ValueError('Для Nav2 запустите web_navigation.launch.py вместо web_mapping.launch.py')
        if mode not in ('mapping', 'nav_slam', 'nav_map'):
            raise ValueError('Неизвестный режим карты')
        map_path = None
        if mode == 'nav_map':
            map_path = self.maps_dir / (map_name(selected_map) + '.yaml')
            if not map_path.is_file():
                raise ValueError('Сначала сохраните или выберите существующую карту')
        with self.lock:
            self.bridge.invoke('cancel')
            self.stop_auto()
            keep_slam = mode in ('mapping', 'nav_slam') and self.slam_process is not None and self.slam_process.poll() is None
            stop_process(self.process)
            self.process = None
            if not keep_slam:
                stop_process(self.slam_process)
                self.slam_process = None
            self.bridge.invoke('stack', mode='transition', preserve_map=keep_slam)
            # DDS may retain endpoints briefly after a node exits during startup.
            deadline = time.monotonic() + 25.0
            while True:
                conflicts = self.bridge.invoke('nodes', allowed=['slam_toolbox'] if keep_slam else [])['conflicts']
                if not conflicts or time.monotonic() >= deadline:
                    break
                time.sleep(0.2)
            if conflicts:
                self.error = 'Остановите другой SLAM/Nav2 launch: ' + ', '.join(conflicts)
                raise ValueError(self.error)
            command = ['ros2', 'launch', 'ugv01_room_explore', 'navigation.launch.py', 'mode:=' + mode]
            if map_path:
                command.append('map:=' + str(map_path))
            self.bridge.invoke('stack', mode=mode, preserve_map=keep_slam)
            if mode in ('mapping', 'nav_slam'):
                if not keep_slam:
                    self.slam_process = self.spawn(
                        ['ros2', 'launch', 'ugv01_room_explore', 'navigation.launch.py', 'mode:=mapping'], 'slam.log')
                if mode == 'nav_slam':
                    self.process = self.spawn(command + ['start_slam:=false'], 'navigation.log')
            else:
                self.process = self.spawn(command, 'navigation.log')
            self.mode, self.selected_map, self.error = mode, selected_map if map_path else None, ''

    def stop_auto(self):
        stop_process(self.auto_process)
        self.auto_process = None

    def drive(self, mode):
        with self.lock:
            self.bridge.invoke('cancel')
            self.stop_auto()
            if mode == 'AUTO':
                self.auto_process = self.spawn(['bash', '-lc', self.auto_command], 'auto.log')
            self.bridge.invoke('drive', mode=mode)

    def stop(self):
        # Close the velocity gate before waiting for a process or transition lock.
        self.bridge.invoke('cancel')
        with self.lock:
            self.stop_auto()

    def save_map(self, name):
        name = map_name(name)
        with self.lock:
            if self.mode not in ('mapping', 'nav_slam'):
                raise ValueError('Сохранение карты доступно в SLAM')
            if not self.bridge.read().get('map_available'):
                raise ValueError('Нет карты для сохранения')
            target = self.maps_dir / name
            if any(target.with_suffix(suffix).exists() for suffix in ('.yaml', '.pgm', '.png')):
                raise ValueError('Карта с таким именем уже есть; выберите другое имя')
            result = subprocess.run([
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(target),
                '--ros-args', '-p', 'save_map_timeout:=10.0',
                '-p', 'map_subscribe_transient_local:=true',
            ], capture_output=True, text=True, timeout=25)
            if result.returncode or not target.with_suffix('.yaml').is_file():
                raise RuntimeError('Не удалось сохранить карту: ' + (result.stderr or result.stdout)[-1500:])
            return {'ok': True, 'map': name}

    def status(self):
        return {
            'managed': self.managed, 'stack_mode': self.mode, 'map': self.selected_map,
            'auto_running': self.auto_process is not None and self.auto_process.poll() is None,
            'auto_pid': self.auto_process.pid if self.auto_process is not None else None,
            'stack_pid': self.process.pid if self.process is not None else None,
            'slam_pid': self.slam_process.pid if self.slam_process is not None else None,
            'stack_running': (self.slam_process is not None and self.slam_process.poll() is None
                              if self.mode == 'mapping' else self.process is not None and self.process.poll() is None),
            'error': self.error,
        }

    def monitor(self):
        while not self.closed.wait(1.0):
            with self.lock:
                if self.auto_process is not None and self.auto_process.poll() is not None:
                    self.bridge.invoke('cancel')
                    stop_process(self.auto_process)
                    self.auto_process = None
                    self.error = 'AUTO завершился; см. auto.log'
                if self.process is not None and self.process.poll() is not None:
                    self.bridge.invoke('stack', mode='error')
                    stop_process(self.process)
                    self.process = None
                    self.error = 'SLAM/Nav2 завершился; см. navigation.log'
                if self.slam_process is not None and self.slam_process.poll() is not None:
                    self.bridge.invoke('stack', mode='error')
                    stop_process(self.slam_process)
                    self.slam_process = None
                    self.error = 'SLAM завершился; см. slam.log'

    def close(self):
        self.closed.set()
        try:
            self.bridge.invoke('cancel')
        except Exception:
            # Reap software children even if the ROS executor/context has died.
            pass
        with self.lock:
            self.stop_auto()
            stop_process(self.process)
            stop_process(self.slam_process)
            self.process = None
            self.slam_process = None
        self.launcher.shutdown(wait=True)
