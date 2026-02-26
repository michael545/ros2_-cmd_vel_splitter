import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile

_VALIDATORS = {
    'input_topic': (Parameter.Type.STRING, lambda v: bool(v), 'must be a non-empty string'),
    'output_topics': (Parameter.Type.STRING_ARRAY, lambda v: v and all(v), 'must be a non-empty list of non-empty strings'),
    'queue_depth': (Parameter.Type.INTEGER, lambda v: v >= 1, 'must be an integer >= 1'),
}


class CmdVelSplitter(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_splitter')
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topics', ['/mcb7/cmd_vel', '/mcb31/cmd_vel'])
        self.declare_parameter('queue_depth', 10)
        self._sub = None
        self._pubs = []
        self._apply(self._cfg(), initial=True)
        self.add_on_set_parameters_callback(self._on_params)

    def _cfg(self):
        g = self.get_parameter
        return g('input_topic').value, list(g('output_topics').value), int(g('queue_depth').value)

    def _apply(self, cfg, initial=False):
        inp, outs, depth = cfg
        for p in self._pubs:
            self.destroy_publisher(p)
        if self._sub:
            self.destroy_subscription(self._sub)
        qos = QoSProfile(depth=depth)
        self._sub = self.create_subscription(Twist, inp, self._cb, qos)
        self._pubs = [self.create_publisher(Twist, t, qos) for t in outs]
        self._cfg_cache = cfg
        label = "Startup" if initial else "Updated"
        self.get_logger().info(f"{label}: input='{inp}', outputs={outs}, queue_depth={depth}")

    def _on_params(self, params):
        inp, outs, depth = self._cfg_cache
        mapping = {'input_topic': lambda v: (v, outs, depth),
                   'output_topics': lambda v: (inp, list(v), depth),
                   'queue_depth': lambda v: (inp, outs, int(v))}
        for p in params:
            if p.name in _VALIDATORS:
                typ, check, reason = _VALIDATORS[p.name]
                if p.type_ != typ or not check(p.value):
                    return SetParametersResult(successful=False, reason=f'{p.name} {reason}')
            if p.name in mapping:
                inp, outs, depth = mapping[p.name](p.value)
        new_cfg = (inp, outs, depth)
        if new_cfg != self._cfg_cache:
            self._apply(new_cfg)
        return SetParametersResult(successful=True)

    def _cb(self, msg):
        for p in self._pubs:
            p.publish(msg)


def main(args=None):
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
