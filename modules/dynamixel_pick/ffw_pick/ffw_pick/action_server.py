"""ROS adapter for the existing analytic pick engine; TaskManager owns sequencing."""
import threading
import signal

import rclpy
from ffw_vision_msgs.action import PickDynamixel
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from ffw_pick import pick_moveit
from ffw_pick.resources import PickResources


class PickRun:
    def __init__(self, handle, stopping=None, resources=None):
        self.handle = handle
        self.resources = resources
        self.stopping = stopping or threading.Event()
        self.move_to_observe = handle.request.move_to_observe
        self.hold = handle.request.hold_before_motion
        self.released = threading.Event()  # set by /dynamixel_pick/release while holding
        self.lock = threading.Lock()
        self.holding = False
        self.motion_started = False
        self.stop_confirmed = True
        self.error_code = ''
        self.message = ''

    def check_cancel(self):
        if self.handle.is_cancel_requested or self.stopping.is_set():
            raise KeyboardInterrupt

    def begin_hold(self):
        with self.lock:
            self.holding = True

    def end_hold(self):
        with self.lock:
            self.holding = False

    def release(self):
        """True when this run was holding (it moves on), else False."""
        with self.lock:
            if not self.holding or self.released.is_set():
                return False
            self.released.set()
            return True

    def phase(self, name, detail):
        self.handle.publish_feedback(PickDynamixel.Feedback(phase=name, detail=detail))


class PickServer(Node):
    def __init__(self):
        super().__init__('dynamixel_pick_backend')
        self.lock = threading.Lock()
        self.stopping = threading.Event()
        self.busy = False
        self.faulted = False
        self.run = None  # the request now executing (release target)
        self.resources = PickResources()
        for name in ('arm', 'config', 'task', 'calibration', 'table', 'urdf'):
            self.declare_parameter(name + '_file', '')
        self.server = ActionServer(
            self, PickDynamixel, '/dynamixel_pick/execute', self.execute,
            goal_callback=self.accept,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.create_service(Trigger, '/dynamixel_pick/release', self.on_release,
                            callback_group=ReentrantCallbackGroup())

    def on_release(self, request, response):
        with self.lock:
            run = self.run
        response.success = run is not None and run.release()
        response.message = 'released' if response.success else 'no pick is holding'
        return response

    def accept(self, request):
        with self.lock:
            if self.stopping.is_set() or self.busy or self.faulted or (request.move_to_observe and not request.execute) or \
                    (request.hold_before_motion and request.move_to_observe):  # a held pick must not move first
                return GoalResponse.REJECT
            self.busy = True
        return GoalResponse.ACCEPT

    def execute(self, handle):
        run = PickRun(handle, self.stopping, self.resources)
        with self.lock:
            self.run = run
        # The shared demo URDF owns both TCP extension and finger collisions.
        args = ['--home-lift', '0', '--no-finger-boxes']
        if handle.request.execute:
            args.append('--run')
        for name in ('arm', 'config', 'task', 'calibration', 'table', 'urdf'):
            value = self.get_parameter(name + '_file').value
            if value:
                args.extend(['--' + name, value])
        result = PickDynamixel.Result()
        try:
            code = pick_moveit.main(args, hooks=run)
            result.success = code == 0
        except BaseException as error:
            # Do not report success or permit another run after an unknown active execution.
            run.message = f'{type(error).__name__}: {error}'
            run.error_code = 'BACKEND_ERROR'
            self.get_logger().error(run.message)
        result.holding = False  # Compatibility field: physical grasp is not verified.
        result.motion_started = run.motion_started
        result.stop_confirmed = run.stop_confirmed
        result.error_code = '' if result.success else (run.error_code or 'PICK_FAILED')
        result.message = run.message or (
            'Close and lift completed; physical grasp is not verified' if result.success and handle.request.execute
            else 'Plan ready' if result.success
            else 'Pick failed; see dynamixel_pick_backend log for the planning diagnostic')
        if not run.stop_confirmed:
            result.success = False
            result.error_code = 'STOP_UNCONFIRMED'
        result.retryable = (not result.success and not run.motion_started and
                            run.stop_confirmed and result.error_code in
                            ('OBJECT_NOT_FOUND', 'PICK_PLAN_FAILED', 'HOLD_TIMEOUT'))
        with self.lock:
            self.run = None
            self.faulted = not run.stop_confirmed or result.error_code == 'SCENE_UNCONFIRMED'
            self.busy = False
        if handle.is_cancel_requested:
            handle.canceled(result)
        elif result.success:
            handle.succeed(result)
        else:
            handle.abort(result)
        return result

    def destroy_node(self):
        self.resources.close()
        return super().destroy_node()


def main():
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = PickServer()
    signal.signal(signal.SIGINT, lambda *_: node.stopping.set())
    signal.signal(signal.SIGTERM, lambda *_: node.stopping.set())
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.stopping.is_set():
            executor.spin_once(timeout_sec=0.1)
    finally:
        node.stopping.set()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
