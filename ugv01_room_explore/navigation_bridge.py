"""Persistent ROS interface for the HTTP dashboard. All ROS work runs in one executor."""
from concurrent.futures import Future
import math
import queue
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String
from std_srvs.srv import Empty as EmptyService
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def finite_pose(data):
    values = tuple(float(data[k]) for k in ('x', 'y', 'yaw'))
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Координаты и угол должны быть конечными числами')
    return values


def map_cell(info, x, y):
    """World -> grid, including a rotated OccupancyGrid origin."""
    dx, dy = x - info.origin.position.x, y - info.origin.position.y
    a = yaw_of(info.origin.orientation)
    gx = (math.cos(a) * dx + math.sin(a) * dy) / info.resolution
    gy = (-math.sin(a) * dx + math.cos(a) * dy) / info.resolution
    return math.floor(gx), math.floor(gy)


class NavigationBridge(Node):
    LIFECYCLE = ('controller_server', 'planner_server', 'behavior_server',
                 'bt_navigator', 'velocity_smoother', 'amcl')

    def __init__(self):
        super().__init__('ugv01_dashboard')
        self.commands = queue.Queue()
        self.snapshot_lock = threading.Lock()
        self.snapshot = {}
        self.stack_mode = 'mapping'
        self.drive_mode = 'JOY'
        self.actual_mode = None
        self.nav_selected_at = 0.0
        self.epoch = 0
        self.stack_generation = 0
        self.goal_handle = None
        self.goal_pending = False
        self.goal_state = 'idle'
        self.goal = None
        self.distance = None
        self.nav_error = ''
        self.map_msg = None
        self.scan_time = self.odom_time = self.amcl_time = 0.0
        self.amcl_cov = None
        self.global_requested = False
        self.service_pending = False
        self.global_future = None
        self.states = {}
        self.state_pending = set()
        self.mode_pub = self.create_publisher(String, '/ugv01/mode', 10)
        self.heartbeat_pub = self.create_publisher(Empty, '/ugv01/nav_heartbeat', 10)
        self.create_subscription(Empty, '/ugv01/stop', lambda _msg: self.do_cancel(), 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/ugv01/robot_pose', 10)
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.global_client = self.create_client(EmptyService, '/reinitialize_global_localization')
        self.update_client = self.create_client(EmptyService, '/request_nomotion_update')
        self.state_clients = {name: self.create_client(GetState, f'/{name}/get_state')
                              for name in self.LIFECYCLE}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/ugv01/mode_state', self.on_actual_mode, latched)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(LaserScan, '/ldlidar_node/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, latched)
        self.create_timer(0.02, self.drain)
        self.create_timer(0.2, self.update_snapshot)
        self.create_timer(1.0, self.localize_tick)
        self.create_timer(1.0, self.lifecycle_tick)

    def on_map(self, msg):
        self.map_msg = msg

    def on_actual_mode(self, msg):
        self.actual_mode = msg.data

    def on_scan(self, _msg):
        self.scan_time = time.monotonic()

    def on_odom(self, _msg):
        self.odom_time = time.monotonic()

    def on_amcl(self, msg):
        self.amcl_cov = [float(msg.pose.covariance[i]) for i in (0, 7, 35)]
        self.amcl_time = time.monotonic()

    def invoke(self, operation, **args):
        reply = Future()
        self.commands.put((operation, args, reply))
        try:
            return reply.result(timeout=8.0)
        except TimeoutError:
            reply.cancel()  # never execute a queued request after its HTTP timeout
            if operation == 'goal':
                self.commands.put(('cancel', {}, Future()))
            raise RuntimeError('ROS не ответил вовремя; команда отменена')

    def drain(self):
        for _ in range(20):
            try:
                operation, args, reply = self.commands.get_nowait()
            except queue.Empty:
                break
            if reply.cancelled():
                continue
            try:
                result = getattr(self, 'do_' + operation)(reply=reply, **args)
                if result is not None and not reply.done():
                    reply.set_result(result)
            except Exception as exc:
                if not reply.done():
                    reply.set_exception(exc)

    def read(self):
        with self.snapshot_lock:
            return dict(self.snapshot)

    def publish_mode(self, mode):
        self.drive_mode = mode
        if mode == 'NAV':
            self.nav_selected_at = time.monotonic()
        self.mode_pub.publish(String(data=mode))

    def do_cancel(self, reply=None):
        self.epoch += 1
        self.goal_pending = False
        self.publish_mode('JOY')
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.goal_state = 'canceled' if self.goal_state in ('pending', 'navigating') else self.goal_state
        self.distance = None
        return {'ok': True}

    def do_drive(self, mode, reply=None):
        if mode not in ('JOY', 'AUTO'):
            raise ValueError('Invalid drive mode')
        self.do_cancel()
        self.publish_mode(mode)
        return {'ok': True}

    def do_stack(self, mode, preserve_map=False, reply=None):
        self.do_cancel()
        self.stack_generation += 1
        self.stack_mode = mode
        if not preserve_map:
            self.map_msg = None
            self.tf_buffer.clear()
        self.amcl_cov = None
        self.amcl_time = 0.0
        self.states.clear()
        self.state_pending.clear()
        self.global_requested = False
        self.service_pending = False
        self.global_future = None
        self.goal_state = 'idle'
        self.goal = None
        self.nav_error = ''
        self.update_snapshot()
        return {'ok': True}

    def lifecycle_tick(self):
        if self.stack_mode not in ('nav_map', 'nav_slam'):
            return
        generation = self.stack_generation
        for name, client in self.state_clients.items():
            if not client.service_is_ready():
                self.states.pop(name, None)
                continue
            if name in self.state_pending:
                continue
            self.state_pending.add(name)
            future = client.call_async(GetState.Request())
            def done(f, name=name, generation=generation):
                self.state_pending.discard(name)
                if self.stack_generation == generation:
                    try:
                        self.states[name] = (f.result().current_state.id, time.monotonic())
                    except Exception:
                        self.states.pop(name, None)
            future.add_done_callback(done)

    def active(self, name):
        state, stamp = self.states.get(name, (0, 0.0))
        return state == 3 and time.monotonic() - stamp < 3.0

    def do_nodes(self, allowed=(), reply=None):
        owners = {'slam_toolbox', 'amcl', 'map_server', 'controller_server', 'planner_server', 'bt_navigator'}
        return {'conflicts': sorted(owners.intersection(self.get_node_names()).difference(allowed))}

    def localize_tick(self):
        if self.stack_mode != 'nav_map' or not self.active('amcl'):
            return
        if not self.global_requested and self.map_msg is not None and self.global_client.service_is_ready():
            self.do_global()
        if self.global_requested and not self.service_pending and self.update_client.service_is_ready():
            self.service_pending = True
            future = self.update_client.call_async(EmptyService.Request())
            future.add_done_callback(lambda _f: setattr(self, 'service_pending', False))

    def do_global(self, reply=None):
        if self.stack_mode != 'nav_map' or not self.active('amcl') or not self.global_client.service_is_ready():
            raise ValueError('Глобальный поиск доступен после запуска AMCL на готовой карте')
        if self.global_future is not None and not self.global_future.done():
            raise ValueError('Глобальный поиск уже запрашивается')
        self.do_cancel()
        self.amcl_cov = None
        self.global_requested = True
        future = self.global_client.call_async(EmptyService.Request())
        self.global_future = future
        generation = self.stack_generation
        def done(f):
            if generation != self.stack_generation:
                if reply is not None and not reply.done():
                    reply.set_result({'ok': False, 'error': 'Режим карты изменён'})
                return
            try:
                f.result()
                self.amcl_cov = None
                result = {'ok': True}
            except Exception as exc:
                self.global_requested = False
                result = {'ok': False, 'error': str(exc)}
            if reply is not None and not reply.done():
                reply.set_result(result)
        future.add_done_callback(done)
        return None

    def do_initialpose(self, x, y, yaw, reply=None):
        if self.stack_mode != 'nav_map' or not self.active('amcl'):
            raise ValueError('Начальная поза доступна после запуска AMCL на готовой карте')
        if self.global_future is not None and not self.global_future.done():
            raise ValueError('Дождитесь завершения запроса глобального поиска и повторите начальную позу')
        x, y, yaw = finite_pose({'x': x, 'y': y, 'yaw': yaw})
        self.validate_point(x, y, allow_unknown=False)
        self.do_cancel()
        self.amcl_cov = None
        self.global_requested = True
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.15 ** 2
        msg.pose.covariance[35] = math.radians(15) ** 2
        self.initial_pub.publish(msg)
        return {'ok': True}

    def validate_point(self, x, y, allow_unknown=True):
        if self.map_msg is None:
            raise ValueError('Дождитесь карты')
        info = self.map_msg.info
        gx, gy = map_cell(info, x, y)
        if not (0 <= gx < info.width and 0 <= gy < info.height):
            raise ValueError('Точка за границами текущей карты; сначала расширьте карту в JOY/AUTO')
        cell = self.map_msg.data[gy * info.width + gx]
        if cell >= 65 or (cell < 0 and not allow_unknown):
            raise ValueError('Выберите свободное место на карте')

    def current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
            age = (self.get_clock().now() - Time.from_msg(transform.header.stamp)).nanoseconds / 1e9
            if age > 2.0:
                return None
            t, q = transform.transform.translation, transform.transform.rotation
            msg = PoseStamped()
            msg.header = transform.header
            msg.pose.position.x, msg.pose.position.y = t.x, t.y
            msg.pose.orientation = q
            self.pose_pub.publish(msg)
            return {'x': t.x, 'y': t.y, 'yaw': yaw_of(q)}
        except TransformException:
            return None

    def readiness(self, pose):
        now = time.monotonic()
        if self.stack_mode not in ('nav_map', 'nav_slam'):
            return 'Выберите Nav2 + SLAM или Nav2 + карта'
        if self.map_msg is None:
            return 'Ожидание /map'
        if now - self.scan_time > 2.0:
            return 'Нет свежих данных лидара'
        if now - self.odom_time > 1.0:
            return 'Нет свежей одометрии'
        if pose is None:
            return 'Ожидание TF map → base_footprint'
        if self.stack_mode == 'nav_map':
            cov = self.amcl_cov
            if cov is None or now - self.amcl_time > 4.0:
                return 'AMCL: задайте начальную позу или дождитесь глобального поиска'
            if not all(math.isfinite(v) and 0 <= v <= limit
                       for v, limit in zip(cov, (0.25, 0.25, 0.30))):
                return 'AMCL: высокая неопределённость; уточните позу и направление'
        for name in self.LIFECYCLE[:-1]:
            if not self.active(name):
                return 'Ожидание Nav2: ' + name
        if not self.action.server_is_ready():
            return 'Ожидание NavigateToPose'
        return ''

    def do_goal(self, x, y, yaw, reply=None):
        x, y, yaw = finite_pose({'x': x, 'y': y, 'yaw': yaw})
        reason = self.readiness(self.current_pose())
        if reason:
            raise ValueError(reason)
        if self.goal_handle is not None or self.goal_pending:
            raise ValueError('Сначала отмените текущую цель')
        self.validate_point(x, y, allow_unknown=self.stack_mode == 'nav_slam')
        self.do_cancel()
        token = self.epoch
        self.goal = {'x': x, 'y': y, 'yaw': yaw}
        self.goal_state, self.goal_pending, self.nav_error = 'pending', True, ''
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        def feedback(msg):
            if token == self.epoch:
                self.distance = float(msg.feedback.distance_remaining)
        future = self.action.send_goal_async(goal, feedback_callback=feedback)
        def accepted(f):
            try:
                handle = f.result()
                if token != self.epoch or reply.cancelled():
                    if handle.accepted:
                        handle.cancel_goal_async()
                    if not reply.done():
                        reply.set_result({'ok': False, 'error': 'Цель отменена'})
                    return
                self.goal_pending = False
                if not handle.accepted:
                    self.goal_state = 'rejected'
                    reply.set_result({'ok': False, 'error': 'Nav2 отклонил цель'})
                    return
                self.goal_handle, self.goal_state = handle, 'navigating'
                self.publish_mode('NAV')
                handle.get_result_async().add_done_callback(lambda f: self.finished(f, token))
                if not reply.done():
                    reply.set_result({'ok': True, 'goal': self.goal})
            except Exception as exc:
                if token == self.epoch:
                    self.do_cancel()
                    self.goal_state, self.nav_error = 'failed', str(exc)
                if not reply.done():
                    reply.set_exception(exc)
        future.add_done_callback(accepted)
        return None

    def finished(self, future, token):
        if token != self.epoch:
            return
        try:
            result = future.result()
            state = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_CANCELED: 'canceled',
                     GoalStatus.STATUS_ABORTED: 'failed'}.get(result.status, 'failed')
            self.nav_error = getattr(result.result, 'error_msg', '')
        except Exception as exc:
            state, self.nav_error = 'failed', str(exc)
        self.do_cancel()
        self.goal_state = state

    def update_snapshot(self):
        pose = self.current_pose()
        reason = self.readiness(pose)
        if self.drive_mode == 'NAV' and self.actual_mode is not None and self.actual_mode != 'NAV' and time.monotonic() - self.nav_selected_at > 1.0:
            reason = 'Диспетчер отключил NAV; отправьте цель заново'
        # Loss of sensors, localization or lifecycle readiness closes the NAV gate.
        if self.drive_mode == 'NAV' and reason:
            self.do_cancel()
            self.goal_state, self.nav_error = 'failed', reason
        if self.drive_mode == 'NAV':
            self.heartbeat_pub.publish(Empty())
        snapshot = {
            'stack_mode': self.stack_mode, 'drive_mode': self.drive_mode,
            'actual_mode': self.actual_mode,
            'ready': not reason, 'reason': reason, 'pose': pose,
            'map_available': self.map_msg is not None,
            'localization_covariance': self.amcl_cov, 'goal_state': self.goal_state,
            'goal': self.goal, 'distance_remaining': self.distance, 'error': self.nav_error,
            'lifecycle': {name: self.active(name) for name in self.LIFECYCLE},
        }
        with self.snapshot_lock:
            self.snapshot = snapshot


class RosRuntime:
    def __init__(self):
        rclpy.init(args=[])
        self.node = NavigationBridge()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def close(self):
        self.node.invoke('cancel')
        self.executor.shutdown(timeout_sec=3.0)
        self.thread.join(timeout=3.0)
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
