"""Action lifecycle with a fake engine; no controllers or cameras are contacted."""
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from ffw_vision_msgs.action import PickDynamixel
from std_srvs.srv import Trigger
from ffw_pick.action_server import PickServer
from ffw_pick import pick_moveit
from ffw_pick.resources import PickResources


class TestPickAction(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.server = PickServer()
        self.client_node = rclpy.create_node('test_pick_client')
        self.client = ActionClient(self.client_node, PickDynamixel, '/dynamixel_pick/execute')
        self.executor = MultiThreadedExecutor(num_threads=3)
        self.executor.add_node(self.server)
        self.executor.add_node(self.client_node)
        self.thread = threading.Thread(target=self.executor.spin)
        self.thread.start()
        self.assertTrue(self.client.wait_for_server(timeout_sec=3))

    def tearDown(self):
        self.server.stopping.set()
        self.executor.shutdown()
        self.thread.join(3)
        self.client_node.destroy_node()
        self.server.destroy_node()
        rclpy.shutdown()

    def wait(self, future):
        deadline = time.monotonic() + 3
        while not future.done() and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertTrue(future.done())
        return future.result()

    def send(self):
        return self.wait(self.client.send_goal_async(
            PickDynamixel.Goal(execute=True, move_to_observe=True)))

    def test_success_does_not_require_or_claim_holding(self):
        def engine(args, hooks):
            self.assertIn('--run', args)
            self.assertIn('--no-finger-boxes', args)
            hooks.motion_started = True
            return 0
        with patch('ffw_pick.pick_moveit.main', engine):
            goal = self.send()
            result = self.wait(goal.get_result_async())
            self.assertEqual(result.status, 4)
            self.assertFalse(result.result.holding)
            self.assertIn("not verified", result.result.message)
            self.assertFalse(result.result.retryable)

    def test_perception_failure_can_retry_but_execution_failure_cannot(self):
        for moved in (False, True):
            def engine(args, hooks):
                hooks.motion_started = moved
                hooks.error_code = 'OBJECT_NOT_FOUND'
                return 1
            with patch('ffw_pick.pick_moveit.main', engine):
                goal = self.send()
                result = self.wait(goal.get_result_async()).result
                self.assertEqual(result.retryable, not moved)

    def test_empty_basket_is_reported_and_not_retryable(self):
        def engine(args, hooks):
            hooks.error_code = 'BASKET_EMPTY'
            hooks.message = 'Basket empty in consecutive observations'
            return 1
        with patch('ffw_pick.pick_moveit.main', engine):
            goal = self.send()
            result = self.wait(goal.get_result_async()).result
            self.assertEqual(result.error_code, 'BASKET_EMPTY')
            self.assertFalse(result.retryable)
            self.assertFalse(result.motion_started)
            self.assertTrue(result.stop_confirmed)

    def test_repeated_requests_share_resources_but_reset_run_state_after_failure(self):
        runs = []

        def engine(args, hooks):
            self.assertIs(hooks.resources, self.server.resources)
            self.assertFalse(hooks.motion_started)
            self.assertTrue(hooks.stop_confirmed)
            self.assertEqual(hooks.error_code, '')
            self.assertEqual(hooks.message, '')
            runs.append(hooks)
            if len(runs) == 1:
                hooks.motion_started = True
                hooks.error_code = 'PICK_PLAN_FAILED'
                hooks.message = 'first request failed after confirmed stop'
                return 1
            return 0

        with patch('ffw_pick.pick_moveit.main', engine):
            first = self.send()
            self.assertEqual(self.wait(first.get_result_async()).status, 6)
            second = self.send()
            result = self.wait(second.get_result_async())
            self.assertEqual(result.status, 4)
            self.assertTrue(result.result.success)
            self.assertNotEqual(runs[0], runs[1])
            self.assertIs(runs[0].resources, runs[1].resources)

    def test_cancel_and_single_goal_arbitration(self):
        entered = threading.Event()
        def engine(args, hooks):
            entered.set()
            while True:
                try:
                    hooks.check_cancel()
                except KeyboardInterrupt:
                    hooks.error_code = 'CANCELLED'
                    return 1
                time.sleep(.01)
        with patch('ffw_pick.pick_moveit.main', engine):
            goal = self.send()
            self.assertTrue(entered.wait(2))
            other = self.send()
            self.assertFalse(other.accepted)
            self.wait(goal.cancel_goal_async())
            result = self.wait(goal.get_result_async())
            self.assertEqual(result.status, 5)
            self.assertTrue(result.result.stop_confirmed)
        with patch('ffw_pick.pick_moveit.main', return_value=0):
            next_goal = self.send()
            self.assertTrue(next_goal.accepted)
            self.assertTrue(self.wait(next_goal.get_result_async()).result.success)

    def test_existing_client_can_request_again_after_backend_restart(self):
        with patch('ffw_pick.pick_moveit.main', return_value=0):
            first = self.send()
            self.assertTrue(self.wait(first.get_result_async()).result.success)
            old_resources = self.server.resources
            self.executor.remove_node(self.server)
            self.server.destroy_node()
            self.server = PickServer()
            self.executor.add_node(self.server)
            self.assertIsNot(self.server.resources, old_resources)
            self.assertTrue(self.client.wait_for_server(timeout_sec=3))
            second = self.send()
            self.assertTrue(second.accepted)
            self.assertTrue(self.wait(second.get_result_async()).result.success)

    def test_retained_service_client_detects_disconnect_and_reconnects(self):
        resources = self.server.resources
        resources.start()
        client = resources.client(Trigger, '/test_pick_prepare')

        def respond(label):
            def callback(request, response):
                response.success = True
                response.message = label
                return response
            return callback

        service = self.client_node.create_service(Trigger, '/test_pick_prepare', respond('first'))
        try:
            for label in ('first', 'restarted'):
                self.assertTrue(client.wait_for_service(timeout_sec=3))
                self.assertIs(resources.client(Trigger, '/test_pick_prepare'), client)
                future = client.call_async(Trigger.Request())
                resources.executor.spin_until_future_complete(future, timeout_sec=3)
                self.assertTrue(future.done())
                self.assertEqual(future.result().message, label)
                if label == 'first':
                    self.client_node.destroy_service(service)
                    service = None
                    deadline = time.monotonic() + 3
                    while client.service_is_ready() and time.monotonic() < deadline:
                        time.sleep(.01)
                    self.assertFalse(client.wait_for_service(timeout_sec=.05))
                    service = self.client_node.create_service(
                        Trigger, '/test_pick_prepare', respond('restarted'))
        finally:
            if service is not None:
                self.client_node.destroy_service(service)

    def test_unknown_scene_latches_backend(self):
        def engine(args, hooks):
            hooks.error_code = 'SCENE_UNCONFIRMED'
            return 1
        with patch('ffw_pick.pick_moveit.main', engine):
            goal = self.send()
            result = self.wait(goal.get_result_async()).result
            self.assertFalse(result.retryable)
            self.assertFalse(self.send().accepted)

    def test_unknown_stop_latches_backend(self):
        def engine(args, hooks):
            hooks.motion_started = True
            hooks.stop_confirmed = False
            raise RuntimeError('lost controller result')
        with patch('ffw_pick.pick_moveit.main', engine):
            goal = self.send()
            result = self.wait(goal.get_result_async()).result
            self.assertEqual(result.error_code, 'STOP_UNCONFIRMED')
            self.assertFalse(self.send().accepted)


class TestPickResources(unittest.TestCase):
    def setUp(self):
        self.factories = {}
        for target in ('rclpy.init', 'rclpy.context.Context', 'rclpy.create_node',
                       'rclpy.executors.SingleThreadedExecutor', 'rclpy.action.ActionClient'):
            patcher = patch(target)
            self.factories[target] = patcher.start()
            self.addCleanup(patcher.stop)
        self.resources = PickResources()
        self.addCleanup(self.resources.close)
        self.node = self.factories['rclpy.create_node'].return_value

    def test_transport_reused_and_request_subscriptions_removed_even_on_error(self):
        clients, actions, states = [], [], []
        hooks = SimpleNamespace(resources=self.resources)

        def engine(argv, hooks, resources):
            resources.start()
            clients.append(resources.client(object, '/plan'))
            actions.append(resources.action(object, '/execute'))
            states.append({})
            resources.subscribe(object, '/observation', states[-1].update, 1)
            if argv:
                raise argv[0]
            return 0

        with patch('ffw_pick.pick_moveit._main', engine):
            self.assertEqual(pick_moveit.main([], hooks), 0)
            for error in (RuntimeError('failed'), KeyboardInterrupt()):
                with self.assertRaises(type(error)):
                    pick_moveit.main([error], hooks)
                self.assertEqual(self.resources.subscriptions, [])
            self.assertEqual(pick_moveit.main([], hooks), 0)
        self.factories['rclpy.init'].assert_called_once()
        self.factories['rclpy.create_node'].assert_called_once()
        self.factories['rclpy.executors.SingleThreadedExecutor'].assert_called_once()
        self.node.create_client.assert_called_once()
        self.factories['rclpy.action.ActionClient'].assert_called_once()
        self.assertTrue(all(client is clients[0] for client in clients))
        self.assertTrue(all(action is actions[0] for action in actions))
        self.assertEqual(self.node.destroy_subscription.call_count, 4)
        callbacks = [call.args[2] for call in self.node.create_subscription.call_args_list]
        callbacks[0]({'old_pose': 1})
        callbacks[-1]({'new_pose': 2})
        self.assertEqual(states[-1], {'new_pose': 2})
        self.node.destroy_node.assert_not_called()

    def test_context_shutdown_discards_old_transport(self):
        self.resources.start()
        self.resources.client(object, '/plan')
        action = self.resources.action(object, '/execute')
        old_context = self.resources.context
        old_node = self.resources.node
        old_context.ok.return_value = False
        new_context, new_node = Mock(), Mock()
        self.factories['rclpy.context.Context'].return_value = new_context
        self.factories['rclpy.create_node'].return_value = new_node
        self.resources.start()
        action.destroy.assert_called_once()
        old_node.destroy_node.assert_called_once()
        old_context.try_shutdown.assert_called_once()
        self.assertIs(self.resources.context, new_context)
        self.assertIs(self.resources.node, new_node)
        self.assertEqual(self.resources.clients, {})
        self.assertEqual(self.resources.actions, {})

    def test_command_line_owns_and_closes_its_resources_on_error(self):
        with patch('ffw_pick.pick_moveit.PickResources') as factory:
            with patch('ffw_pick.pick_moveit._main', side_effect=RuntimeError('setup failed')):
                with self.assertRaises(RuntimeError):
                    pick_moveit.main([])
            factory.return_value.end_request.assert_called_once()
            factory.return_value.close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
