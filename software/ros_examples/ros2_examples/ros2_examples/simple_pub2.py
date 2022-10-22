import rclpy
from rclpy.node import Node
from ros2_examples_interfaces.msg import SimplePubSub

class SimplePublisher2(Node):
	def __init__(self):
		super().__init__('simple_publisher2')
		self.publisher = self.create_publisher(SimplePubSub, 'topic', 10)
		self.timer = self.create_timer(1, self.timer_callback)
		self.i = 0

	def timer_callback(self):
		data = SimplePubSub()
		data.msg_num = self.i
		data.msg = f"Hello World: {self.i}"
		data.sqrt = self.i ** 0.5
		data.is_square = data.sqrt == int(data.sqrt)
		self.publisher.publish(data)
		self.get_logger().info(f"Publishing: '{data}'")
		self.i += 1


def main(args=None):
	rclpy.init(args=args)
	simple_pub2 = SimplePublisher2()
	rclpy.spin(simple_pub2)
	simple_pub2.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
    main()
