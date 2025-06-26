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

    def __enter__(self):
        self.INFO(f"Successful initialization.")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if exc_type: return False
        del self


# ros2 pkg create zjros2 --build-type ament_python --dependencies rclpy
def main():
    with NodeBase("demo") as node:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
