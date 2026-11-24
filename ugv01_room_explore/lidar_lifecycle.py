"""Bring the LiDAR to active state using persistent lifecycle service clients."""
import argparse
import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.executors import ExternalShutdownException


def ensure_active(node, name='/ldlidar_node', timeout=90.0, retry_delay=.5):
    name = '/' + name.strip('/')
    state_client = node.create_client(GetState, name + '/get_state')
    change_client = node.create_client(ChangeState, name + '/change_state')
    deadline = time.monotonic() + timeout

    def call(client, request):
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not client.wait_for_service(timeout_sec=min(.5, remaining)):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future,
            timeout_sec=max(0., min(10., deadline - time.monotonic())))
        if not future.done():
            future.cancel()
            return None
        return future.result()

    try:
        while rclpy.ok(context=node.context) and time.monotonic() < deadline:
            reply = call(state_client, GetState.Request())
            if reply is not None:
                state = reply.current_state.id
                if state == State.PRIMARY_STATE_ACTIVE:
                    node.get_logger().info(name + ' is active')
                    return True
                if state == State.PRIMARY_STATE_FINALIZED:
                    node.get_logger().error(name + ' is finalized; restart its driver')
                    return False
                transition = {
                    State.PRIMARY_STATE_UNCONFIGURED: Transition.TRANSITION_CONFIGURE,
                    State.PRIMARY_STATE_INACTIVE: Transition.TRANSITION_ACTIVATE,
                }.get(state)
                if transition is not None:
                    request = ChangeState.Request()
                    request.transition.id = transition
                    result = call(change_client, request)
                    if result is None or not result.success:
                        node.get_logger().warning(name + ' transition not ready; retrying')
            time.sleep(min(retry_delay, max(0., deadline - time.monotonic())))
        node.get_logger().error(name + ' did not become active before the startup timeout')
        return False
    finally:
        node.destroy_client(state_client)
        node.destroy_client(change_client)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('node', nargs='?', default='/ldlidar_node')
    args = parser.parse_args()
    rclpy.init(args=[])
    node = rclpy.create_node('ugv01_lidar_startup')
    success = False
    try:
        success = ensure_active(node, args.node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if success else 1


if __name__ == '__main__':
    raise SystemExit(main())
