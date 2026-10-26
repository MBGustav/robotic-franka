"""Command-line test tool for the Franka gripper (real or simulated).

    ros2 run aided_building_gripper gripper_cli homing
    ros2 run aided_building_gripper gripper_cli open
    ros2 run aided_building_gripper gripper_cli move 0.03 [speed]
    ros2 run aided_building_gripper gripper_cli grasp 0.02 20 [speed] [eps]
    ros2 run aided_building_gripper gripper_cli stop

Widths in m, force in N, speed in m/s, eps (inner = outer) in m.
"""

import sys

import rclpy
from franka_msgs.action import Grasp, Homing, Move
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger

NS = '/franka_gripper'
MAX_WIDTH = 0.08


def usage():
    print(__doc__)
    return 1


def call_action(node, action_type, name, goal, timeout=20.0):
    client = ActionClient(node, action_type, f'{NS}/{name}')
    if not client.wait_for_server(timeout_sec=5.0):
        print(f'Action server {NS}/{name} not available')
        return False
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    handle = future.result()
    if handle is None or not handle.accepted:
        print('Goal rejected')
        return False
    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=timeout)
    if result_future.result() is None:
        print('Timed out waiting for the result')
        return False
    result = result_future.result().result
    print('success' if result.success else f'failed: {result.error}')
    return result.success


def main(args=None):
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    if not argv:
        return usage()

    rclpy.init(args=args)
    node = Node('gripper_cli')
    cmd, params = argv[0], [float(v) for v in argv[1:]]
    ok = False
    try:
        if cmd == 'homing':
            ok = call_action(node, Homing, 'homing', Homing.Goal())
        elif cmd in ('open', 'move'):
            goal = Move.Goal()
            goal.width = MAX_WIDTH if cmd == 'open' else params[0]
            goal.speed = params[1] if len(params) > 1 else 0.05
            ok = call_action(node, Move, 'move', goal)
        elif cmd == 'grasp':
            goal = Grasp.Goal()
            goal.width, goal.force = params[0], params[1]
            goal.speed = params[2] if len(params) > 2 else 0.05
            eps = params[3] if len(params) > 3 else 0.005
            goal.epsilon.inner = goal.epsilon.outer = eps
            ok = call_action(node, Grasp, 'grasp', goal)
        elif cmd == 'stop':
            client = node.create_client(Trigger, f'{NS}/stop')
            if client.wait_for_service(timeout_sec=5.0):
                future = client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
                ok = future.result() is not None and future.result().success
                print('stopped' if ok else 'stop failed')
            else:
                print(f'Service {NS}/stop not available')
        else:
            usage()
    except (IndexError, ValueError):
        usage()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
