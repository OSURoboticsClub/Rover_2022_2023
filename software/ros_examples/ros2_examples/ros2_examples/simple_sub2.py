import rclpy
from rclpy.node import Node
from ros2_examples_interfaces.msg import SimplePubSub

class SimpleSubscriber2(Node):
	def __init__(self):
		super().__init__('simple_subscriber2')
		self.subscription = self.create_subscription(SimplePubSub, 'topic', self.listener_callback, 10)

	def listener_callback(self, data):
		self.get_logger().info(f"I received: '{data}'")


def main(args=None):
	rclpy.init(args=args)
	simple_sub2 = SimpleSubscriber2()
	rclpy.spin(simple_sub2)
	simple_sub2.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
