from ctypes import *
import rclpy
from rclpy.node import Node

from rover2_arm_interface.msg import ArmControlMessage, ArmStatusMessage

#create cdll instance/load shared library for simplemotion
simplemotion = CDLL("libsimplemotionv2.so")

#####################################
# Global Variables
#####################################
DEFAULT_PORT = "/dev/rover/ttyARM"
DEFAULT_RELATIVE_POS_CONTROL_TOPIC = "control/relative"
DEFAULT_ARM_STATUS_TOPIC = "status"
NODE_NAME = "rover_arm"

############ Simplemotion specific vars and defines ###############
#add defines from headers (DLL cannot access defines)
#view simplemotion_defs.h and simplemotion.h for more details
SM_ABSOLUTE_SETPOINT= 551
SM_SERIAL_ENC_OFFSET = 575
SMP_SYSTEM_CONTROL_RESTART = 2
SMP_SYSTEM_CONTROL = 554
SMP_FAULTS = 552
SMP_CONTROL_BITS1 = 2533
SMP_CB1_ENABLE = (1<<0)
SMP_ACTUAL_POSITION_FB = 903
SMP_STATUS = 553
SMP_SYSTEM_CONTROL_SAVECFG = 1
#addresses are placeholder numbers until we actually figure out addr in granity
#numbers for counts_per_rev/min and max revs are placeholders until we figure out actual values
#main_pitch
main_pitch_address = c_uint8(1)
main_pitch_counts_per_rev = c_int32(0)
main_pitch_min_rev  = c_double(0)
main_pitch_max_rev = c_double(0)

main_pitch_min_rev_counts = c_int32(0)
main_pitch_max_rev_counts = c_int32(0)

#wrist_pitch
wrist_pitch_address = c_uint8(2)
wrist_pitch_count_per_rev = c_int32(0)
wrist_pitch_min_rev = c_double(0)
wrist_pitch_max_rev = c_double(0)

wrist_pitch_min_rev_counts = c_int32(0);
wrist_pitch_max_rev_counts = c_int32(0);

#wrist_roll
wrist_roll_address = c_uint8(3)
wrist_roll_counts_per_rev = c_double(1638400)

#####################################
# ArmControl Class Definition
#####################################
class ArmControl(Node):
	def __init__(self):
		super().__init__(NODE_NAME)

		self.arm_port = self.declare_parameter("~port", DEFAULT_PORT).value
		print(self.arm_port)

		self.arm_bus_handle = simplemotion.smOpenBus(self.arm_port)

		if(self.arm_bus_handle < 0):
			print("Could not connect to arm")
			self.arm_successfully_connected = False
		else:
			self.arm_successfully_connected = True

		#declare params for pubs/subs
		self.relative_position_control_subscriber_topic = self.declare_parameter("~relative_position_control_topic", DEFAULT_RELATIVE_POS_CONTROL_TOPIC).value
		#self.arm_status_topic = self.declare_parameter("~arm_control_status_topic", DEFAULT_ARM_STATUS_TOPIC).value

		#create publishers and subscribers
		self.relative_position_control_subscriber = self.create_subscription(ArmControlMessage, self.relative_position_control_subscriber_topic, self.relative_position_callback, 1)
		#self.arm_control_status_publisher = self.create_publisher(ArmStatusMessage, self.arm_status_topic, 1)

		#set counts for contraints
		main_pitch_min_rev_counts = main_pitch_min_rev.value * main_pitch_counts_per_rev.value
		c_int32(int(main_pitch_min_rev_counts))
		main_pitch_max_rev_counts = main_pitch_max_rev.value * main_pitch_counts_per_rev.value
		c_int32(int(main_pitch_max_rev_counts))

		wrist_pitch_min_rev_counts = wrist_pitch_min_rev.value * wrist_pitch_count_per_rev.value
		c_int32(int(wrist_pitch_min_rev_counts))
		wrist_pitch_max_rev_counts = wrist_pitch_max_rev.value * wrist_pitch_count_per_rev.value
		c_int32(int(wrist_pitch_max_rev_counts))

		#setup control messages
		self.relative_pos_control_message = ArmControlMessage()

		#initialize vars (these were originally "Private" class members in the cpp version of this)
		self.new_positions_received = False
		self.should_reset = False
		self.should_clear_faults = False
		self.should_calibrate = False

		#main pitch variables
		self.main_pitch_set_position = c_int32(0)
		self.main_pitch_current_position = c_int32(0)
		self.main_pitch_comm_state = c_int32(0)
		self.main_pitch_faults = c_int32(0)
		self.main_pitch_status = c_int32(0)

		#wrist pitch variables
		self.wrist_pitch_set_position = c_int32(0)
		self.wrist_pitch_current_position = c_int32(0)
		self.wrist_pitch_comm_state = c_int32(0)
		self.wrist_pitch_faults = c_int32(0)
		self.wrist_pitch_status = c_int32(0)

		#wrist roll variables
		self.wrist_roll_set_position = c_int32(0)
		self.wrist_roll_current_position = c_int32(0)
		self.wrist_roll_comm_state = c_int32(0)
		self.wrist_roll_faults = c_int32(0)
		self.wrist_roll_status = c_int32(0)


	def set_joint_positions(self):
		if self.new_positions_received is True:
			set_main_pitch_position()
			set_wrist_positions()
			self.new_positions_received = False

	def set_main_pitch_position(self):
		simplemotion.smSetParameter(self.arm_bus_handle, main_pitch_address, SMP_ABSOLUTE_SETPOINT, self.main_pitch_set_position)

	def set_wrist_positions(self):
		simplemotion.smSetParameter(self.arm_bus_handle, wrist_pitch_address, SMP_ABSOLUTE_SETPOINT, self.wrist_pitch_set_position)
		simplemotion.smSetParameter(self.arm_bus_handle, wrist_roll_address, SMP_ABSOLUTE_SETPOINT, self.wrist_roll_set_position)

	def constrain_set_positions(self):
		#bound set position between min and max constraints so no overtravelling occurs
		self.main_pitch_set_position =  min(max(self.main_pitch_set_position, main_pitch_min_rev_counts), main_pitch_max_rev_counts)
		self.wrist_pitch_set_position = min(max(self.wrist_pitch_set_position, wrist_pitch_min_rev_counts), wrist_pitch_max_rev_counts)

	def reset_controllers(self):
		if self.should_reset is True:
			simplemotion.smSetParameter(self.arm_bus_handle, main_pitch_address, SMP_SYSTEM_CONTROL, SMP_SYSTEM_CONTROL_RESTART)
			simplemotion.smSetParameter(self.arm_bus_handle, wrist_pitch_address, SMP_SYSTEM_CONTROL, SMP_SYSTEM_CONTROL_RESTART)
			simplemotion.smSetParameter(self.arm_bus_handle, wrist_roll_address, SMP_SYSTEM_CONTROL, SMP_SYSTEM_CONTROL_RESTART)

			self.arm_successfully_connected = False
			self.should_reset = False

	def clear_faults(self):
		if self.should_clear_faults is True:
			simplemotion.smSetParameters(self.arm_bus_handle, main_pitch_address, SMP_FAULTS, 0)
			simplemotion.smSetParameters(self.arm_bus_handle, main_pitch_address, SMP_CONTROL_BITS1, SMP_CB1_ENABLE)

			simplemotion.smSetParameters(self.arm_bus_handle, wrist_pitch_address, SMP_FAULTS, 0)
			simplemotion.smSetParameters(self.arm_bus_handle, wrist_pitch_address, SMP_CONTROL_BITS1, SMP_CB1_ENABLE)

			simplemotion.smSetParameters(self.arm_bus_handle, wrist_roll_address, SMP_FAULTS, 0)
			simplemotion.smSetParameters(self.arm_bus_handle, wrist_roll_address, SMP_CONTROL_BITS1, SMP_CB1_ENABLE)

			self.should_clear_faults = False

	def relative_position_callback(self):
		self.main_pitch_set_position += self.relative_pos_control_message.base * self.main_pitch_counts_per_rev
		self.wrist_roll_set_positiion += self.relative_pos_control_message.wrist_roll * self.wrist_roll_counts_per_rev
		self.wrist_pitch_set_position += self.relative_pos_control_message.wrist_pitch * self.wrist_pitch_count_per_rev

		self.constrain_set_positions()

		self.should_clear_faults = self.relative_pos_control_message.clear_faults
		self.should_reset = self.relative_pos_control_message.should_reset

		if self.should_reset is False:
			self.new_positions_received = True

	def main_loop(self):
		if(self.arm_successfully_connected):
			self.clear_faults()
			self.set_joint_positions()
			self.reset_controllers()

def main(args=None):
	rclpy.init(args=args)
	arm_control = ArmControl()
	rclpy.spin(arm_control)
	arm_control.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()
