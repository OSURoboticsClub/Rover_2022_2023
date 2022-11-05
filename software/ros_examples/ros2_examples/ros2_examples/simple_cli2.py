import rclpy
from rclpy.node import Node
from ros2_examples_interfaces.srv import SimpleSrvCli
import time

class SimpleClient2(Node):
	def __init__(self):
		super().__init__("simple_cli2")
		self.cli = self.create_client(SimpleSrvCli, "simple_srv_cli")
		while not self.cli.wait_for_service(timeout_sec=1):
			self.get_logger().info("Waiting for service")
		self.req = SimpleSrvCli.Request()
		self.i = 0

	def send_request(self):
		self.req.num = self.i
		self.i += 1
		self.future = self.cli.call_async(self.req)
		rclpy.spin_until_future_complete(self, self.future)
		resp = self.future.result()
		self.get_logger().info(f"Response received: {resp}")


def main(args=None):
	rclpy.init(args=args)
	simple_cli2 = SimpleClient2()
	while True:
		try:
			response = simple_cli2.send_request()
			time.sleep(1)
		except KeyboardInterrupt:
			break
	simple_cli2.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
    main()
