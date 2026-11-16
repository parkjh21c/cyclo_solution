"""Reusable ROS transport with subscriptions scoped to one pick request."""


class PickResources:
    def __init__(self):
        self.context = None
        self.node = None
        self.executor = None
        self.clients = {}
        self.actions = {}
        self.publishers = {}
        self.subscriptions = []

    def start(self, signal_handlers=False):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.signals import SignalHandlerOptions

        if self.context is not None and not self.context.ok():
            self.close()
        reused = self.node is not None
        if not reused:
            self.context = Context()
            try:
                options = None if signal_handlers else SignalHandlerOptions.NO
                rclpy.init(context=self.context, signal_handler_options=options)
                self.node = rclpy.create_node('pick_moveit', context=self.context)
                self.executor = SingleThreadedExecutor(context=self.context)
                self.executor.add_node(self.node)
            except BaseException:
                self.close()
                raise
        self.node.get_logger().info('pick resources: ' + ('reused' if reused else 'created'))

    def subscribe(self, *args):
        subscription = self.node.create_subscription(*args)
        self.subscriptions.append(subscription)
        return subscription

    def client(self, service_type, name):
        key = (service_type, name)
        if key not in self.clients:
            self.clients[key] = self.node.create_client(service_type, name)
        return self.clients[key]

    def action(self, action_type, name):
        from rclpy.action import ActionClient

        key = (action_type, name)
        if key not in self.actions:
            self.actions[key] = ActionClient(self.node, action_type, name)
        return self.actions[key]

    def publisher(self, message_type, name, qos):
        key = (message_type, name)
        if key not in self.publishers:
            self.publishers[key] = self.node.create_publisher(message_type, name, qos)
        return self.publishers[key]

    def end_request(self):
        for subscription in self.subscriptions:
            self.node.destroy_subscription(subscription)
        self.subscriptions.clear()

    def close(self):
        self.end_request()
        if self.executor is not None:
            self.executor.shutdown()
        for action in self.actions.values():
            action.destroy()
        if self.node is not None:
            self.node.destroy_node()
        if self.context is not None:
            self.context.try_shutdown()
        self.clients.clear()
        self.actions.clear()
        self.publishers.clear()
        self.context = self.node = self.executor = None
