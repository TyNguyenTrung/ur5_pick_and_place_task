import rclpy
from rclpy.node import Node

class HelloNode(Node):
    def __init__(self):
        super().__init__('hello_node')
        self.get_logger().info('hello')

def test_fcn1():
    print('done1')

def main(args=None):
    rclpy.init(args=args)
    node = HelloNode()
    rclpy.spin_once(node)  # only spin once so it prints and exits
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
