import rclpy.node
import sys


class NodeBase(rclpy.node.Node):

    def __init__(self, name):
        rclpy.init(args=sys.argv)
        super().__init__(name)
        # log methods
        self.DEBUG = self.get_logger().debug
        self.INFO = self.get_logger().info
        self.WARN = self.get_logger().warn
        self.ERROR = self.get_logger().error
        self.FATAL = self.get_logger().fatal

    def __bool__(self):
        return rclpy.ok()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if exc_type: return False
        del self


# ros2 pkg create zjros2 --build-type ament_python --dependencies rclpy
def main():
    with NodeBase("demo") as node:
        try:
            while node:
                node.INFO("pending...")
                rclpy.spin_once(node, timeout_sec=1.0)
        except KeyboardInterrupt:
            pass
