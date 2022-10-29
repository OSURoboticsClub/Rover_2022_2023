import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from ros2_examples_interfaces.action import SimpleAction
import time

def is_prime(num, primes):
	i = 0
	sqrt = int(num**0.5)
	while i < len(primes) and primes[i] <= sqrt:
		if num % primes[i] == 0:
			return False
		i += 1

	return True

class SimpleActionServer2(Node):
	def __init__(self):
		super().__init__("simple_action_srv2")
		self.srvv = ActionServer(self, SimpleAction, 'simple_action', self.execute_callback)

	def execute_callback(self, goal):
		self.get_logger().info("Executing goal...")
		num_primes = goal.request.num_primes
		result = SimpleAction.Result()
		feedback = SimpleAction.Feedback()
		feedback.progress = 0.0

		primes = [2, 3]
		if num_primes <= 2:
			goal.succeed()
			result.primes = primes[:num_primes]
			return result

		i = 2
		candidate = 6
		while i < num_primes:
			if is_prime(candidate - 1, primes):
				primes.append(candidate - 1)
				i += 1
				if i == num_primes:
					break
			if is_prime(candidate + 1, primes):
				primes.append(candidate + 1)
				i += 1
			candidate += 6
			feedback.progress = float(100 * i / num_primes)
			self.get_logger().info(f"Feedback: {feedback.progress}% done - {i}/{num_primes}")
			goal.publish_feedback(feedback)
			time.sleep(0.1)

		goal.succeed()
		result.primes = primes
		return result

def main(args=None):
	rclpy.init(args=args)
	simple_action_srv2 = SimpleActionServer2()
	rclpy.spin(simple_action_srv2)
	simple_action_srv2.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()
