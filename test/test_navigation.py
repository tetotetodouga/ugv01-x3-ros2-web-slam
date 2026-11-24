"""Software regression checks. Run with ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1."""
from concurrent.futures import Future
import math
import os
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, String

from ugv01_room_explore.cmd_vel_mode_manager import CmdVelModeManager
from ugv01_room_explore.navigation_bridge import NavigationBridge, finite_pose, map_cell
from ugv01_room_explore.stack_supervisor import StackSupervisor, map_name


@pytest.fixture(scope='module', autouse=True)
def ros():
    if os.environ.get('ROS_DOMAIN_ID') != '91' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        pytest.skip('Run with ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1; hardware domain is excluded')
    rclpy.init(args=[])
    yield
    rclpy.shutdown()


@pytest.fixture
def mux():
    node = CmdVelModeManager()
    node.pub = Mock()
    yield node
    node.destroy_node()


def twist(x):
    msg = Twist()
    msg.linear.x = float(x)
    return msg


def test_nav_is_separate_from_auto_and_joy(mux):
    mux.on_nav_cmd(twist(.1))
    mux.on_mode(String(data='NAV'))
    mux.on_timer()
    assert mux.pub.publish.call_args.args[0].linear.x == 0  # discard pre-switch buffer
    mux.on_auto_cmd(twist(.2))
    mux.on_joy_cmd(twist(.3))
    mux.on_nav_cmd(twist(.08))
    mux.on_timer()
    assert mux.pub.publish.call_args.args[0].linear.x == .08


def test_stop_cannot_be_undone_by_late_nav_velocity(mux):
    mux.on_mode(String(data='NAV'))
    mux.on_nav_cmd(twist(.1))
    mux.on_stop(Empty())
    mux.on_nav_cmd(twist(.12))
    mux.on_timer()
    assert mux.mode == 'JOY'
    assert mux.pub.publish.call_args.args[0].linear.x == 0


def test_nav_timeout(mux):
    mux.on_mode(String(data='NAV'))
    mux.on_nav_cmd(twist(.1))
    mux.last_nav_time -= 1.0
    mux.on_timer()
    assert mux.pub.publish.call_args.args[0].linear.x == 0


def test_nav_heartbeat_loss_latches_stop(mux):
    mux.on_mode(String(data='NAV'))
    mux.on_nav_cmd(twist(.1))
    mux.last_nav_heartbeat -= 2.0
    mux.on_timer()
    assert mux.mode == 'JOY'
    mux.on_nav_heartbeat(Empty())
    mux.on_nav_cmd(twist(.1))
    mux.on_timer()
    assert mux.mode == 'JOY'
    assert mux.pub.publish.call_args.args[0].linear.x == 0


def test_auto_waits_for_new_auto_command(mux):
    mux.on_mode(String(data='AUTO'))
    mux.on_nav_cmd(twist(.1))
    assert mux.mode == 'JOY'
    mux.on_auto_cmd(twist(.06))
    mux.on_timer()
    assert mux.mode == 'AUTO'
    assert mux.pub.publish.call_args.args[0].linear.x == .06


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -float('inf')])
def test_reject_nonfinite_goals(bad):
    with pytest.raises(ValueError):
        finite_pose({'x': bad, 'y': 0, 'yaw': 0})


def test_rotated_map_origin():
    grid = OccupancyGrid()
    grid.info.resolution = .5
    grid.info.origin.position.x = 10.0
    grid.info.origin.position.y = 20.0
    grid.info.origin.orientation.z = math.sin(math.pi / 4)
    grid.info.origin.orientation.w = math.cos(math.pi / 4)
    assert map_cell(grid.info, 9.25, 20.75) == (1, 1)


@pytest.fixture
def bridge():
    node = NavigationBridge()
    yield node
    node.destroy_node()


def test_unknown_goal_allowed_only_for_live_slam(bridge):
    grid = OccupancyGrid()
    grid.info.width = grid.info.height = 2
    grid.info.resolution = 1.0
    grid.info.origin.orientation.w = 1.0
    grid.data = [0, -1, 100, 0]
    bridge.map_msg = grid
    bridge.validate_point(1.5, .5, allow_unknown=True)
    with pytest.raises(ValueError):
        bridge.validate_point(1.5, .5, allow_unknown=False)
    with pytest.raises(ValueError):
        bridge.validate_point(.5, 1.5)
    with pytest.raises(ValueError):
        bridge.validate_point(-.1, .5)


def test_stop_during_goal_acceptance_cancels_late_handle(bridge, monkeypatch):
    pending = Future()
    monkeypatch.setattr(bridge.action, 'send_goal_async', Mock(return_value=pending))
    monkeypatch.setattr(bridge, 'readiness', lambda _: '')
    monkeypatch.setattr(bridge, 'current_pose', lambda: {})
    monkeypatch.setattr(bridge, 'validate_point', lambda *a, **k: None)
    reply = Future()
    bridge.do_goal(1, 2, 0, reply=reply)
    bridge.do_cancel()
    handle = Mock(accepted=True)
    pending.set_result(handle)
    handle.cancel_goal_async.assert_called_once()
    assert bridge.drive_mode == 'JOY'
    assert not reply.result()['ok']


def test_http_timeout_cannot_accept_goal_later(bridge, monkeypatch):
    pending = Future()
    monkeypatch.setattr(bridge.action, 'send_goal_async', Mock(return_value=pending))
    monkeypatch.setattr(bridge, 'readiness', lambda _: '')
    monkeypatch.setattr(bridge, 'current_pose', lambda: {})
    monkeypatch.setattr(bridge, 'validate_point', lambda *a, **k: None)
    reply = Future()
    bridge.do_goal(1, 2, 0, reply=reply)
    reply.cancel()
    handle = Mock(accepted=True)
    pending.set_result(handle)
    handle.cancel_goal_async.assert_called_once()
    assert bridge.drive_mode == 'JOY'


def test_no_saved_map_localization_without_confidence(bridge):
    bridge.stack_mode = 'nav_map'
    assert bridge.readiness(None)
    bridge.amcl_cov = [100, 100, 100]
    assert bridge.readiness({'x': 0, 'y': 0, 'yaw': 0})


def test_receiving_old_odom_does_not_refresh_sensor_age(bridge):
    import time
    msg = Odometry()
    msg.header.stamp = bridge.get_clock().now().to_msg()
    msg.header.stamp.sec -= 2
    bridge.on_odom(msg)
    assert time.monotonic() - bridge.odom_time >= 1.9


def test_odom_tf_timer_holds_only_fresh_telemetry(monkeypatch):
    from ugv01_room_explore.ugv_odom import Odom
    monkeypatch.setattr('ugv01_room_explore.ugv_odom.serial.Serial', Mock())
    clock = [10.0]
    monkeypatch.setattr('ugv01_room_explore.ugv_odom.time', SimpleNamespace(monotonic=lambda: clock[0]))
    node = Odom()
    try:
        sent = [call.args[0] for call in node.ser.write.call_args_list]
        assert b'{"T":142,"cmd":50}\n' in sent
        node.publish = Mock()
        node.publish_timer()
        node.publish.assert_not_called()
        node.process({'odl': 0, 'odr': 0})
        node.publish_timer()
        assert node.publish.call_count == 1
        clock[0] += 0.02
        node.publish_timer()
        assert node.publish.call_count == 2
        clock[0] += 0.3
        node.publish_timer()
        assert node.publish.call_count == 2
    finally:
        node.destroy_node()


@pytest.mark.parametrize('name', ['../map', 'a/b', '', 'map;touch /tmp/x', '/tmp/map', None])
def test_map_names_cannot_escape_storage(name):
    with pytest.raises(ValueError):
        map_name(name)


def test_switching_live_modes_keeps_slam_process(tmp_path, monkeypatch):
    bridge = Mock()
    bridge.invoke.return_value = {'conflicts': []}
    supervisor = StackSupervisor(bridge, True, tmp_path / 'maps', tmp_path / 'logs', 'auto')
    slam = SimpleNamespace(poll=lambda: None)
    supervisor.slam_process = slam
    stopped = []
    monkeypatch.setattr('ugv01_room_explore.stack_supervisor.stop_process', stopped.append)
    monkeypatch.setattr(supervisor, 'spawn', Mock())
    supervisor.switch('nav_slam')
    assert supervisor.slam_process is slam
    assert slam not in stopped
    assert supervisor.spawn.call_count == 1
    assert 'start_slam:=false' in supervisor.spawn.call_args.args[0]
