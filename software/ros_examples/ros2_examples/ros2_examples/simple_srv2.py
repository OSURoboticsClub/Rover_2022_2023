import rclpy
from rclpy.node import Node
from ros2_examples_interfaces.srv import SimpleSrvCli
from math import e as euler

class SimpleService2(Node):
	def __init__(self):
		super().__init__("simple_srv2")
		self.srv = self.create_service(SimpleSrvCli, 'simple_srv_cli', self.service_callback)

	def service_callback(self, request, response):
		response.is_even = request.num % 2 == 0
		response.exp = euler ** request.num
		self.get_logger().info(f"Incoming request: {request}")
		return response

def main(args=None):
	rclpy.init(args=args)
	simple_srv2 = SimpleService2()
	rclpy.spin(simple_srv2)
	rclpy.shutdown()

if __name__ == "__main__":
	main()
