#!/usr/bin/env python3
"""Wait until Nav2 waypoint_follower is lifecycle-active (whole stack is up)."""

import time

import rclpy
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState


def wait_for_waypoint_follower_active(
    executor,
    node,
    *,
    timeout_sec: float = 180.0,
    poll_sec: float = 0.25,
) -> bool:
    """
    Poll /waypoint_follower/get_state until PRIMARY_STATE_ACTIVE.

    waypoint_follower is the last node in lifecycle_manager_navigation; when it is active,
    /follow_waypoints exists and the stack is ready for missions.
    """
    client = node.create_client(GetState, '/waypoint_follower/get_state')
    deadline = time.time() + timeout_sec
    print('Waiting for Nav2 lifecycle: /waypoint_follower -> active...')

    while rclpy.ok() and time.time() < deadline:
        if client.service_is_ready():
            break
        executor.spin_once(timeout_sec=poll_sec)
    else:
        print(
            f'Timed out waiting for /waypoint_follower/get_state service ({timeout_sec:.0f}s). '
            'Is bringup running with nav:=true?'
        )
        return False

    while rclpy.ok() and time.time() < deadline:
        request = GetState.Request()
        future = client.call_async(request)
        executor.spin_until_future_complete(future, timeout_sec=5.0)
        if not future.done():
            executor.spin_once(timeout_sec=poll_sec)
            continue
        try:
            response = future.result()
        except Exception as e:
            print(f'GetState call failed: {e}')
            executor.spin_once(timeout_sec=poll_sec)
            continue
        if response is None:
            executor.spin_once(timeout_sec=poll_sec)
            continue
        if response.current_state.id == State.PRIMARY_STATE_ACTIVE:
            print('Nav2 /waypoint_follower is active; safe to send missions.')
            return True
        executor.spin_once(timeout_sec=poll_sec)

    print(
        f'Timed out after {timeout_sec:.0f}s: /waypoint_follower never reached active. '
        'Check: ros2 lifecycle get /bt_navigator ; ros2 lifecycle get /waypoint_follower'
    )
    return False
