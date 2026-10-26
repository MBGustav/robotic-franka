"""Simulated Franka Hand server for Gazebo.

Exposes the same interface as the real ``franka_gripper`` node
(namespace ``/franka_gripper``):

    actions : ~/homing (franka_msgs/Homing)
              ~/move   (franka_msgs/Move)
              ~/grasp  (franka_msgs/Grasp)
              ~/gripper_action (control_msgs/GripperCommand)
    service : ~/stop   (std_srvs/Trigger)
    topic   : ~/joint_states (sensor_msgs/JointState)

so code written against it runs unchanged on the real robot.

Implementation: finger setpoints are ramped at the requested speed and sent
to a JointGroupPositionController on ``fr3_finger_joint1`` (the second finger
is a mimic joint). Width = 2 * finger_joint1.

Limitation: Gazebo position control cannot regulate the grasp force. The
``force`` field is validated but not applied; once the fingers stall on an
object they are held slightly inside the contact to keep the object.
"""

import threading
import time

import rclpy
from control_msgs.action import GripperCommand
from franka_msgs.action import Grasp, Homing, Move
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger


class Preempted(Exception):
    pass


class SimGripperServer(Node):

    def __init__(self):
        super().__init__('franka_gripper')

        p = self.declare_parameter
        self.joint_names = list(p(
            'joint_names', ['fr3_finger_joint1', 'fr3_finger_joint2']).value)
        self.command_topic = p(
            'command_topic', '/fr3_gripper_controller/commands').value
        self.max_width = float(p('max_width', 0.08).value)          # [m]
        self.max_speed = float(p('max_speed', 0.1).value)           # [m/s]
        self.max_force = float(p('max_force', 70.0).value)          # [N]
        self.default_speed = float(p('default_speed', 0.1).value)
        self.eps_inner = float(p('default_grasp_epsilon.inner', 0.005).value)
        self.eps_outer = float(p('default_grasp_epsilon.outer', 0.005).value)
        self.rate = float(p('control_rate', 100.0).value)           # [Hz]
        self.move_tolerance = float(p('move_tolerance', 0.002).value)
        self.stall_speed = float(p('stall_speed', 0.002).value)     # [m/s]
        self.stall_time = float(p('stall_time', 0.3).value)         # [s]
        self.squeeze = float(p('grasp_squeeze', 0.002).value)       # [m] width
        # Gazebo (DART) sticks a prismatic joint that sits exactly on its limit,
        # so the fully open setpoint stays this far below max_width.
        self.limit_margin = float(p('limit_margin', 0.001).value)   # [m] width
        self.state_rate = float(p('state_publish_rate', 15.0).value)

        self._lock = threading.Lock()
        self._width = None        # measured width [m]
        self._speed = 0.0         # measured width rate [m/s]
        self._setpoint = None     # last commanded width [m]
        self._goal_token = 0      # incremented to preempt the running goal

        cb = ReentrantCallbackGroup()
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, self.command_topic, 10)
        self.state_pub = self.create_publisher(JointState, '~/joint_states', 10)
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_states, 10,
            callback_group=cb)
        self.create_timer(1.0 / self.state_rate, self._publish_state,
                          callback_group=cb)
        self.create_service(Trigger, '~/stop', self._on_stop, callback_group=cb)

        common = dict(goal_callback=self._on_goal,
                      cancel_callback=lambda _: CancelResponse.ACCEPT,
                      callback_group=cb)
        ActionServer(self, Homing, '~/homing', self._exec_homing, **common)
        ActionServer(self, Move, '~/move', self._exec_move, **common)
        ActionServer(self, Grasp, '~/grasp', self._exec_grasp, **common)
        ActionServer(self, GripperCommand, '~/gripper_action',
                     self._exec_gripper_command, **common)

        self.get_logger().info(
            f'Simulated gripper ready, commanding {self.command_topic}')

    # ------------------------------------------------------------ state
    def _on_joint_states(self, msg):
        if self.joint_names[0] not in msg.name:
            return
        i = msg.name.index(self.joint_names[0])
        with self._lock:
            self._width = 2.0 * msg.position[i]
            if len(msg.velocity) > i:
                self._speed = 2.0 * msg.velocity[i]
            if self._setpoint is None:
                self._setpoint = self._width

    def _publish_state(self):
        with self._lock:
            width = self._width
        if width is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = [width / 2.0, width / 2.0]
        msg.velocity = [0.0, 0.0]
        msg.effort = [0.0, 0.0]
        self.state_pub.publish(msg)

    def _state(self):
        with self._lock:
            return self._width, self._speed

    def _send(self, width):
        width = min(max(width, 0.0), self.max_width - self.limit_margin)
        with self._lock:
            self._setpoint = width
        self.cmd_pub.publish(Float64MultiArray(data=[width / 2.0]))

    # ------------------------------------------------------------ goals
    def _on_goal(self, _goal):
        if self._state()[0] is None:
            self.get_logger().error('No finger joint state yet; rejecting goal')
            return GoalResponse.REJECT
        # A new goal preempts the running one (same as the real gripper).
        with self._lock:
            self._goal_token += 1
        return GoalResponse.ACCEPT

    def _on_stop(self, _request, response):
        with self._lock:
            self._goal_token += 1
            width = self._width
        if width is None:
            response.success = False
            response.message = 'No finger joint state yet'
            return response
        self._send(width)
        response.success = True
        response.message = 'Gripper stopped'
        return response

    def _check(self, token, goal_handle):
        with self._lock:
            preempted = token != self._goal_token
        if preempted or goal_handle.is_cancel_requested:
            raise Preempted()

    def _ramp(self, target, speed, goal_handle, token, feedback, stop_on_stall):
        """Ramp the setpoint toward ``target``. Returns (width, stalled)."""
        speed = min(abs(speed) if speed > 0 else self.default_speed,
                    self.max_speed)
        dt = 1.0 / self.rate
        with self._lock:
            sp = self._setpoint if self._setpoint is not None else self._width
        stalled_since = None
        t_start = time.monotonic()
        timeout = abs(target - sp) / speed + 3.0

        while rclpy.ok():
            self._check(token, goal_handle)
            step = speed * dt
            sp = target if abs(target - sp) <= step else sp + step * (
                1 if target > sp else -1)
            self._send(sp)

            width, rate = self._state()
            if feedback is not None:
                feedback(width)

            if abs(width - target) <= self.move_tolerance:
                return width, False

            # Stall: setpoint already ahead of the fingers and they stopped.
            if stop_on_stall and abs(sp - width) > self.move_tolerance and \
                    abs(rate) < self.stall_speed:
                stalled_since = stalled_since or time.monotonic()
                if time.monotonic() - stalled_since >= self.stall_time:
                    return width, True
            else:
                stalled_since = None

            if time.monotonic() - t_start > timeout:
                return width, True
            time.sleep(dt)
        return self._state()[0], True

    def _franka_feedback(self, goal_handle, fb_type):
        def publish(width):
            fb = fb_type.Feedback()
            fb.current_width = float(width)
            goal_handle.publish_feedback(fb)
        return publish

    def _finish(self, goal_handle, result, success, error=''):
        result.success = success
        result.error = error
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        if error:
            self.get_logger().warn(error)
        return result

    def _token(self):
        with self._lock:
            return self._goal_token

    # ------------------------------------------------------- franka actions
    def _do_move(self, goal_handle, width, speed, fb):
        if not 0.0 <= width <= self.max_width:
            return False, f'Width {width:.4f} m out of range [0, {self.max_width}]'
        final, _ = self._ramp(width, speed, goal_handle, self._token(), fb, True)
        if abs(final - width) > self.move_tolerance:
            self._send(final)  # do not keep pushing against an obstacle
            return False, f'Move blocked at {final:.4f} m (target {width:.4f} m)'
        return True, ''

    def _do_grasp(self, goal_handle, width, speed, force, inner, outer, fb):
        if not 0.0 <= width <= self.max_width:
            return False, f'Width {width:.4f} m out of range [0, {self.max_width}]'
        if not 0.0 <= force <= self.max_force:
            return False, f'Force {force:.1f} N out of range [0, {self.max_force}]'
        # Close until contact (like the real hand), then check the window.
        final, _ = self._ramp(0.0, speed, goal_handle, self._token(), fb, True)
        ok = (width - inner) <= final <= (width + outer)
        if ok:
            self._send(max(final - self.squeeze, 0.0))   # hold the object
            return True, ''
        return False, (f'Grasp failed: measured width {final:.4f} m not in '
                       f'[{width - inner:.4f}, {width + outer:.4f}] m')

    def _run(self, goal_handle, result, job):
        try:
            success, error = job()
        except Preempted:
            result.success = False
            result.error = 'Goal preempted or canceled'
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return result
        return self._finish(goal_handle, result, success, error)

    def _exec_homing(self, goal_handle):
        fb = self._franka_feedback(goal_handle, Homing)
        return self._run(goal_handle, Homing.Result(), lambda: self._do_move(
            goal_handle, self.max_width, self.default_speed, fb))

    def _exec_move(self, goal_handle):
        g = goal_handle.request
        fb = self._franka_feedback(goal_handle, Move)
        return self._run(goal_handle, Move.Result(), lambda: self._do_move(
            goal_handle, g.width, g.speed, fb))

    def _exec_grasp(self, goal_handle):
        g = goal_handle.request
        fb = self._franka_feedback(goal_handle, Grasp)
        return self._run(goal_handle, Grasp.Result(), lambda: self._do_grasp(
            goal_handle, g.width, g.speed, g.force,
            g.epsilon.inner, g.epsilon.outer, fb))

    # ------------------------------------------- control_msgs/GripperCommand
    def _exec_gripper_command(self, goal_handle):
        """Same rule as franka_gripper: width = 2 * position; opening -> move,
        closing -> grasp with max_effort and the default epsilon."""
        cmd = goal_handle.request.command
        target = 2.0 * cmd.position
        current = self._state()[0]
        result = GripperCommand.Result()

        def fb(width):
            f = GripperCommand.Feedback()
            f.position = float(width)
            goal_handle.publish_feedback(f)

        try:
            if target >= current:
                ok, err = self._do_move(goal_handle, target,
                                        self.default_speed, fb)
            else:
                ok, err = self._do_grasp(goal_handle, target,
                                         self.default_speed,
                                         min(cmd.max_effort, self.max_force),
                                         self.eps_inner, self.eps_outer, fb)
        except Preempted:
            ok, err = False, 'Goal preempted or canceled'

        result.position = float(self._state()[0])
        result.reached_goal = ok
        result.stalled = not ok
        if ok:
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            self.get_logger().warn(err)
            goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = SimGripperServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
