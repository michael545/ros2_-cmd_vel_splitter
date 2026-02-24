import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile
from typing import List


class CmdVelSplitter(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_splitter')

        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topics', ['/mcb7/cmd_vel', '/mcb31/cmd_vel'])
        self.declare_parameter('queue_depth', 10)

        self._input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self._output_topics = list(
            self.get_parameter('output_topics').get_parameter_value().string_array_value
        )
        self._queue_depth = int(
            self.get_parameter('queue_depth').get_parameter_value().integer_value
        )

        self._qos_profile = QoSProfile(depth=self._queue_depth)
        self._subscription = None
        self._publishers: List[Publisher] = []

        self._apply_configuration(
            input_topic=self._input_topic,
            output_topics=self._output_topics,
            queue_depth=self._queue_depth,
            initial=True,
        )

        self.add_on_set_parameters_callback(self._on_parameters_changed)

    def _apply_configuration(
        self,
        *,
        input_topic: str,
        output_topics: List[str],
        queue_depth: int,
        initial: bool = False,
    ) -> None:
        """Update subscription and publishers after parameter changes."""
        if not input_topic:
            raise ValueError('input_topic cannot be empty')
        if not output_topics:
            raise ValueError('output_topics must contain at least one topic')
        if queue_depth < 1:
            raise ValueError('queue_depth must be >= 1')

        for publisher in self._publishers:
            self.destroy_publisher(publisher)
        if self._subscription is not None:
            self.destroy_subscription(self._subscription)

        self._qos_profile = QoSProfile(depth=queue_depth)

        self._subscription = self.create_subscription(
            Twist,
            input_topic,
            self._cmd_vel_callback,
            self._qos_profile,
        )

        self._publishers = [
            self.create_publisher(Twist, topic, self._qos_profile)
            for topic in output_topics
        ]

        self._input_topic = input_topic
        self._output_topics = output_topics
        self._queue_depth = queue_depth

        if initial:
            self.get_logger().info(
                f"Startup: listening on '{self._input_topic}', republishing to {self._output_topics}"
            )
        else:
            self.get_logger().info(
                f"Updated: input='{self._input_topic}', outputs={self._output_topics}, queue_depth={self._queue_depth}"
            )

    def _on_parameters_changed(self, params: List[Parameter]) -> SetParametersResult:
        next_input = self._input_topic
        next_outputs = list(self._output_topics)
        next_depth = self._queue_depth

        for param in params:
            if param.name == 'input_topic':
                if param.type_ != Parameter.Type.STRING:
                    return SetParametersResult(
                        successful=False,
                        reason='input_topic must be a string',
                    )
                if not param.value:
                    return SetParametersResult(
                        successful=False,
                        reason='input_topic cannot be empty',
                    )
                next_input = param.value

            elif param.name == 'output_topics':
                if param.type_ != Parameter.Type.STRING_ARRAY:
                    return SetParametersResult(
                        successful=False,
                        reason='output_topics must be a list of strings',
                    )
                if not param.value:
                    return SetParametersResult(
                        successful=False,
                        reason='output_topics must contain at least one topic',
                    )
                if any(not topic for topic in param.value):
                    return SetParametersResult(
                        successful=False,
                        reason='output_topics entries must be non-empty',
                    )
                next_outputs = list(param.value)

            elif param.name == 'queue_depth':
                if param.type_ != Parameter.Type.INTEGER:
                    return SetParametersResult(
                        successful=False,
                        reason='queue_depth must be an integer',
                    )
                if param.value < 1:
                    return SetParametersResult(
                        successful=False,
                        reason='queue_depth must be >= 1',
                    )
                next_depth = int(param.value)

        config_changed = (
            next_input != self._input_topic
            or next_outputs != self._output_topics
            or next_depth != self._queue_depth
        )

        if config_changed:
            try:
                self._apply_configuration(
                    input_topic=next_input,
                    output_topics=next_outputs,
                    queue_depth=next_depth,
                )
            except ValueError as error:
                return SetParametersResult(
                    successful=False,
                    reason=str(error),
                )

        return SetParametersResult(successful=True)

    def _cmd_vel_callback(self, msg: Twist) -> None:
        for publisher in self._publishers:
            publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelSplitter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down cmd_vel_splitter')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
