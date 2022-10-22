import rclpy
from rclpy.node import Node

class HelloRos2(Node):
	def __init__(self):
		super().__init__("hello_ros2")
		self.get_logger().info("Hello ROS2!!!")

def main(args=None):
	rclpy.init(args=args)
	hello_ros2 = HelloRos2()
	rclpy.spin(hello_ros2)
	hello_ros2.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()
