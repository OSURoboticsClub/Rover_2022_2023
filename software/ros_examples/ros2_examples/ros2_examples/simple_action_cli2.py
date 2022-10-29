import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ros2_examples_interfaces.action import SimpleAction

class SimpleActionClient2(Node):
	def __init__(self):
		super().__init__("simple_action_cli2")
		self.cli = ActionClient(self, SimpleAction, 'simple_action')

	def send_goal(self, num_primes):
		goal = SimpleAction.Goal()
		goal.num_primes = num_primes
		self.cli.wait_for_server()
		self.future = self.cli.send_goal_async(goal, feedback_callback=self.feedback_callback)
		self.future.add_done_callback(self.goal_response_callback)

	def goal_response_callback(self, future):
		goal = future.result()
		if not goal.accepted:
			self.get_logger().info("Rejected...")
			return

		self.get_logger().info("Accepted!")
		result_future = goal.get_result_async()
		result_future.add_done_callback(self.get_result_callback)

	def get_result_callback(self, future):
		self.get_logger().info(f"Result: {future.result().result.primes}")
		rclpy.shutdown()

	def feedback_callback(self, msg):
		self.get_logger().info(f"Feedback received: {msg.feedback.progress}% done")

def main(args=None):
	rclpy.init(args=args)
	simple_action_cli2 = SimpleActionClient2()
	simple_action_cli2.send_goal(1000)
	rclpy.spin(simple_action_cli2)


if __name__ == '__main__':
    main()
