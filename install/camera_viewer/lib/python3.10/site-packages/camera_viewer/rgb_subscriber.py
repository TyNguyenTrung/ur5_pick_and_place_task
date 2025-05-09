import rclpy
from rclpy.node import Node
from camera_viewer.py_test_1 import test_fcn1

class HelloNode(Node):
    def __init__(self):
        super().__init__('hello_node')
        self.get_logger().info('hello')

def main(args=None):
    rclpy.init(args=args)
    node = HelloNode()
    rclpy.spin_once(node)  # only spin once so it prints and exits
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
