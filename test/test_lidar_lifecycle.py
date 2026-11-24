"""Retry a rejected LiDAR lifecycle transition without any hardware driver."""
import os
import threading

import pytest
import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.executors import SingleThreadedExecutor

from ugv01_room_explore.lidar_lifecycle import ensure_active


def test_lidar_startup_retries_configuration_then_activates():
    if os.environ.get('ROS_DOMAIN_ID') != '91' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        pytest.skip('Use ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1; exclude hardware domain')
    rclpy.init(args=[])
    server = rclpy.create_node('fake_lidar')
    helper = rclpy.create_node('test_lidar_startup')
    state = State.PRIMARY_STATE_UNCONFIGURED
    transitions = []

    def get_state(request, response):
        response.current_state.id = state
        return response

    def change_state(request, response):
        nonlocal state
        transitions.append(request.transition.id)
        response.success = len(transitions) != 1  # first configure is not ready
        if response.success:
            state = (State.PRIMARY_STATE_INACTIVE if request.transition.id ==
                     Transition.TRANSITION_CONFIGURE else State.PRIMARY_STATE_ACTIVE)
        return response

    server.create_service(GetState, '/fake_lidar/get_state', get_state)
    server.create_service(ChangeState, '/fake_lidar/change_state', change_state)
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    try:
        assert ensure_active(helper, '/fake_lidar', timeout=8., retry_delay=.01)
        assert state == State.PRIMARY_STATE_ACTIVE
        assert transitions == [Transition.TRANSITION_CONFIGURE,
                               Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE]
    finally:
        executor.shutdown(timeout_sec=2.)
        worker.join(timeout=2.)
        helper.destroy_node()
        server.destroy_node()
        rclpy.shutdown()
