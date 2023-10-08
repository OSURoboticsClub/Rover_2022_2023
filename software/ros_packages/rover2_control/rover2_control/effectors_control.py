#!/usr/bin/env python
#####################################
# Imports
#####################################
# Python native imports
import rclpy
from rclpy.node import Node

from time import time, sleep

import serial.rs485
import minimalmodbus

# from std_msgs.msg import UInt8, UInt16

# Custom Imports
from rover2_control_interface.msg import GripperControlMessage, GripperStatusMessage, MiningControlMessage, DrillControlMessage

#####################################
# Global Variables
#####################################
NODE_NAME = "effectors_control"

# ##### Communication Defines #####
DEFAULT_PORT = "/dev/rover/ttyEffectors"
DEFAULT_BAUD = 115200

GRIPPER_NODE_ID = 1 #reflash science board, check lin act board, flash drill board
LINEAR_NODE_ID = 2
DRILL_NODE_ID = 3
SCIENCE_NODE_ID = 4

GRIPPER_TIMEOUT = 0.5
LINEAR_TIMEOUT = 0.3
SCIENCE_TIMEOUT = 0.3
DRILL_TIMEOUT = 0.3

FAILED_GRIPPER_MODBUS_LIMIT = 20
FAILED_LINEAR_MODBUS_LIMIT = 20
FAILED_SCIENCE_MODBUS_LIMIT = 20
FAILED_DRILL_MODBUS_LIMIMT = 20

RX_DELAY = 0.01
TX_DELAY = 0.01

DEFAULT_HERTZ = 40

GRIPPER_CONTROL_SUBSCRIBER_TOPIC = "gripper/control"
GRIPPER_STATUS_PUBLISHER_TOPIC = "gripper/status"
SCIENCE_CONTROL_SUBSCRIBER_TOPIC = "mining/control/compartment"
LINEAR_CONTROL_SUBSCRIBER_TOPIC = "mining/control/linear"
DRILL_CONTROL_SUBSCRIBER_TOPIC = "mining/drill/control"

# ##### Gripper Defines #####
GRIPPER_MODBUS_REGISTERS = {
    "POSITION": 0,
    "HOME": 1,
    "TARGET": 2,
    "SPEED": 3,
    "DIRECTION": 4,
    "LASER": 5,
    "LED": 6,
    "TEMP": 7,
    "DISTANCE": 8,
    "CURRENT": 9,
    "IS_HOMED": 10
}


DEFAULT_GRIPPER_REGISTERS = [
    0,  # No positional update
    0,  # Do not home
    0,  # No target
    0,  # 0 speed
    0,  # No direction
    0,  # No laser
    0,  # Light off
    0,  # 0 temp
    0,  # 0 distance
    0,  # 0 current
    0,  # Not homed
]

GRIPPER_UNIVERSAL_POSITION_MAX = 10000

# ##### Science Defines #####
SCIENCE_MODBUS_REGISTERS = {
    "STEPPER_POSITION": 0,
    "CAMERA_SELECT": 1,
    "LAZER_EN": 2,
    "LIM_SW1": 3,
    "LIM_SW2": 4,
    "LIM_SW3": 5,
    "LIM_SW4": 6
}

DEFAULT_SCIENCE_REGISTERS = [0]*7

LINEAR_MODBUS_REGISTERS = {
    "DIRECTION": 0,
    "SPEED": 1,
    "SLEEP": 2,

    "CURRENT": 3,
    "FAULT": 4,

    "TEMPERATURE": 5
}

DEFAULT_LINEAR_REGISTERS = [0]*6

DRILL_MODBUS_REGISTERS = {
    "DIRECTION": 0,
    "SPEED": 1,
    "SLEEP": 2,

    "CURRENT": 3,
    "FAULT": 4,

    "TEMPERATURE": 5
}

DEFAULT_DRILL_REGISTERS = [0]*6

# ##### Science Defines #####

# ##### Misc Defines #####
NODE_LAST_SEEN_TIMEOUT = 2  # seconds

INT16_MAX = 32767
INT16_MIN = -32768
UINT16_MAX = 65535


#####################################
# DriveControl Class Definition
#####################################
class EffectorsControl(Node):
    EFFECTORS = [
        "GRIPPER",
        "SCIENCE"
    ]

    def __init__(self):
        super().__init__(NODE_NAME)

        self.port = self.declare_parameter("~port", DEFAULT_PORT).value
        self.baud = self.declare_parameter("~baud", DEFAULT_BAUD).value

        
        self.gripper_node_id = self.declare_parameter("~gripper_node_id", GRIPPER_NODE_ID).value
        self.science_node_id = self.declare_parameter("~science_node_id", SCIENCE_NODE_ID).value
        self.drill_node_id = self.declare_parameter("~drill_node_id", DRILL_NODE_ID).value
        self.linear_node_id = self.declare_parameter("~linear_node_id", LINEAR_NODE_ID).value

        
        self.gripper_control_subscriber_topic = self.declare_parameter("~gripper_control_subscriber_topic",
                                                                GRIPPER_CONTROL_SUBSCRIBER_TOPIC).value                                                        
        
        self.gripper_status_publisher_topic = self.declare_parameter("~gripper_status_publisher_topic",
                                                              GRIPPER_STATUS_PUBLISHER_TOPIC).value                                                      

        self.science_control_subscriber_topic = self.declare_parameter("~science_control_subscriber_topic",
                                                             SCIENCE_CONTROL_SUBSCRIBER_TOPIC).value                                                     
        
        self.drill_control_subscriber_topic = self.declare_parameter("~drill_control_subscriber_topic",
                                                               DRILL_CONTROL_SUBSCRIBER_TOPIC).value

        self.linear_control_subscriber_topic = self.declare_parameter("~linear_control_subscriber_topic",
                                                               LINEAR_CONTROL_SUBSCRIBER_TOPIC).value

        self.wait_time = 1.0 / self.declare_parameter("~hertz", DEFAULT_HERTZ).value

        self.gripper_node = None  # type:minimalmodbus.Instrument
        self.science_node = None  # type:minimalmodbus.Instrument
        self.drill_node = None  # type:minimalmodbus.Instrument
        self.linear_node = None

        self.connect_to_nodes()
        # self.check_which_nodes_present()

        # ##### Subscribers #####
        self.gripper_control_subscriber = self.create_subscription(GripperControlMessage, self.gripper_control_subscriber_topic, self.gripper_control_message_received_callback, 1)

        self.science_control_subscriber = self.create_subscription(MiningControlMessage, self.science_control_subscriber_topic, self.science_control_message_received_callback, 1)
        
        self.drill_control_subscriber = self.create_subscription(DrillControlMessage, self.drill_control_subscriber_topic, self.drill_control_message_received_callback, 1)
        
        self.linear_control_subscriber = self.create_subscription(DrillControlMessage, self.linear_control_subscriber_topic, self.linear_control_message_received_callback, 1)

        # ##### Publishers #####
        self.gripper_status_publisher = self.create_publisher(GripperStatusMessage, self.gripper_status_publisher_topic, 1)

        # ##### Misc #####
        self.modbus_nodes_seen_time = time()

        # ##### Mining Variables #####
        self.science_registers = DEFAULT_SCIENCE_REGISTERS
        self.gripper_registers = DEFAULT_GRIPPER_REGISTERS
        self.drill_registers = DEFAULT_DRILL_REGISTERS
        self.linear_registers = DEFAULT_LINEAR_REGISTERS

        self.science_control_message = None  # type:MiningControlMessage
        self.new_science_control_message = False

        self.gripper_control_message = None
        self.new_gripper_control_message = False
        
        self.drill_control_message = None  # type:DrillControlMessage
        self.new_drill_control_message = False

        self.linear_control_message = None
        self.new_linear_control_message = False

        self.camera_control_message = None  # type: CameraControlMessage
        self.new_camera_control_message = False

        self.failed_gripper_modbus_count = 0
        self.failed_science_modbus_count = 0
        self.failed_drill_modbus_count = 0
        self.failed_linear_modbus_count = 0

        self.which_effector = self.EFFECTORS.index("SCIENCE")

        self.gripper_position_status = 0
        self.compartment = 0

        self.timer = self.create_timer(self.wait_time, self.main_loop)

    def __setup_minimalmodbus_for_485(self):
        self.gripper_node.serial = serial.rs485.RS485(self.port, baudrate=self.baud, timeout=GRIPPER_TIMEOUT)
        self.gripper_node.serial.rs485_mode = serial.rs485.RS485Settings(rts_level_for_rx=1, rts_level_for_tx=0, delay_before_rx=RX_DELAY, delay_before_tx=TX_DELAY)

        self.science_node.serial = serial.rs485.RS485(self.port, baudrate=self.baud, timeout=SCIENCE_TIMEOUT)
        self.science_node.serial.rs485_mode = serial.rs485.RS485Settings(rts_level_for_rx=1, rts_level_for_tx=0, delay_before_rx=RX_DELAY, delay_before_tx=TX_DELAY)

        self.drill_node.serial = serial.rs485.RS485(self.port, baudrate=self.baud, timeout=DRILL_TIMEOUT)
        self.drill_node.serial.rs485_mode = serial.rs485.RS485Settings(rts_level_for_rx=1, rts_level_for_tx=0, delay_before_rx=RX_DELAY, delay_before_tx=TX_DELAY)

        self.linear_node.serial = serial.rs485.RS485(self.port, baudrate=self.baud, timeout=LINEAR_TIMEOUT)
        self.linear_node.serial.rs485_mode = serial.rs485.RS485Settings(rts_level_for_rx=1, rts_level_for_tx=0, delay_before_rx=RX_DELAY, delay_before_tx=TX_DELAY)


    def main_loop(self):
        if self.which_effector == self.EFFECTORS.index("GRIPPER"):
            try:
                self.run_arm()
                self.failed_gripper_modbus_count = 0
            except Exception as e:
                print(e)
                self.failed_gripper_modbus_count += 1

            if self.failed_gripper_modbus_count == FAILED_GRIPPER_MODBUS_LIMIT:
                print("Gripper not present. Trying mining.")
                self.which_effector = self.EFFECTORS.index("SCIENCE")

        elif self.which_effector == self.EFFECTORS.index("SCIENCE"):
            try:
                self.run_science()
                self.failed_science_modbus_count = 0
            except Exception as e:
                print(e)
                self.failed_science_modbus_count += 1

            if self.failed_science_modbus_count == FAILED_SCIENCE_MODBUS_LIMIT:
                print("No effectors present. Exiting....")
                self.destroy_node()
                return

    def run_arm(self):
        self.process_gripper_control_message()
        self.process_linear_control_message()
        self.send_gripper_status_message()
        #self.process_linear_control_message()

    def run_science(self):
        #self.process_science_control_message()
        #self.process_drill_control_messages()
        self.process_linear_control_message()

    def connect_to_nodes(self):
        self.gripper_node = minimalmodbus.Instrument(self.port, int(self.gripper_node_id))
        self.science_node = minimalmodbus.Instrument(self.port, int(self.science_node_id))
        self.drill_node = minimalmodbus.Instrument(self.port, int(self.drill_node_id))
        self.linear_node = minimalmodbus.Instrument(self.port, int(self.linear_node_id))

        self.__setup_minimalmodbus_for_485()

    def process_science_control_message(self):
        if self.new_science_control_message:
            self.compartment += (self.science_control_message.compartment - 1 - (self.compartment % 4)) % 4
            self.science_registers[SCIENCE_MODBUS_REGISTERS["STEPPER_POSITION"]] = 465 * self.compartment
            self.science_node.write_registers(0, self.science_registers)
            self.modbus_nodes_seen_time = time()
            self.new_science_control_message = False

    def process_linear_control_message(self):
        if self.new_linear_control_message:
            direction = self.linear_control_message.direction
            self.linear_registers[LINEAR_MODBUS_REGISTERS["DIRECTION"]] = direction
            self.linear_registers[LINEAR_MODBUS_REGISTERS["SLEEP"]] = 1

            #self.science_registers = self.science_node.read_registers(0, len(self.science_registers))
            is_lim_hit = False #any(self.science_registers[SCIENCE_MODBUS_REGISTERS["LIM_SW1"]:SCIENCE_MODBUS_REGISTERS["LIM_SW4"]+1])
            if not is_lim_hit or not direction:
                self.linear_registers[LINEAR_MODBUS_REGISTERS["SPEED"]] = min(self.linear_control_message.speed, UINT16_MAX)
            else:
                self.linear_registers[LINEAR_MODBUS_REGISTERS["SPEED"]] = 0

            self.linear_node.write_registers(0, self.linear_registers)
            self.modbus_nodes_seen_time = time()

            self.new_linear_control_message = False

    def process_gripper_control_message(self):
        if self.new_gripper_control_message:
            if self.gripper_control_message.should_home:
                print("GRIPPER SHOULD_HOME TRUE")
                self.gripper_registers[GRIPPER_MODBUS_REGISTERS["HOME"]] = 1
                self.gripper_node.write_registers(0, self.gripper_registers)

                homing_complete = False

                #gripper_homing_time = time()
                while not homing_complete:
                    #time_elapsed = time() - gripper_homing_time
                    self.gripper_registers = self.gripper_node.read_registers(0, len(GRIPPER_MODBUS_REGISTERS))
                    print(self.gripper_registers)
                    #self.send_gripper_status_message()
                    #print("time elapsed: ", time_elapsed, "homing start time: ", self.gripper_homing_time)

                    #if self.gripper_registers[GRIPPER_MODBUS_REGISTERS["IS_HOMED"]] or time_elapsed >= 1000:
                    #print(GRIPPER_MODBUS_REGISTERS["LED"])
                    #print(GRIPPER_MODBUS_REGISTERS["LASER"])
                    #print(GRIPPER_MODBUS_REGISTERS["IS_HOMED"])
                    #print(self.gripper_registers[10])
                    #print(self.gripper_registers[6])
                    #print(self.gripper_registers[GRIPPER_MODBUS_REGISTERS["LASER"]])
                    #print(self.gripper_registers[GRIPPER_MODBUS_REGISTERS["IS_HOMED"]])

                    if self.gripper_registers[GRIPPER_MODBUS_REGISTERS["IS_HOMED"]]:
                        homing_complete = True
                        print("GRIPPER HOMING COMPLETE")
                        #gripper_homing_time = 0

            else:
                if self.gripper_control_message.toggle_light:
                    self.gripper_registers[GRIPPER_MODBUS_REGISTERS["LED"]] = 0 if self.gripper_registers[GRIPPER_MODBUS_REGISTERS["LED"]] else 1
                    self.gripper_control_message.toggle_light = False

                if self.gripper_control_message.toggle_laser:
                    self.gripper_registers[GRIPPER_MODBUS_REGISTERS["LASER"]] = 0 if self.gripper_registers[GRIPPER_MODBUS_REGISTERS["LASER"]] else 1
                    self.gripper_control_message.toggle_laser = False

                gripper_target = self.gripper_control_message.target

                if -30000 < gripper_target < 30000:
                    new_position = self.gripper_position_status + gripper_target
                    self.gripper_registers[GRIPPER_MODBUS_REGISTERS["TARGET"]] = min(max(new_position, 0), INT16_MAX)
                    self.gripper_registers[GRIPPER_MODBUS_REGISTERS["SPEED"]] = 30000
                    self.gripper_registers[GRIPPER_MODBUS_REGISTERS["DIRECTION"]] = 1

                self.gripper_node.write_registers(0, self.gripper_registers)
                print(self.gripper_registers)

        self.new_gripper_control_message = False

    def send_gripper_status_message(self):
        registers = self.gripper_node.read_registers(0, len(GRIPPER_MODBUS_REGISTERS))

        message = GripperStatusMessage()
        message.position_raw = registers[GRIPPER_MODBUS_REGISTERS["POSITION"]]
        self.gripper_position_status = message.position_raw
        message.temp = registers[GRIPPER_MODBUS_REGISTERS["TEMP"]]
        message.light_on = bool(registers[GRIPPER_MODBUS_REGISTERS["LED"]])
        message.laser_on = bool(registers[GRIPPER_MODBUS_REGISTERS["LASER"]])
        message.current = registers[GRIPPER_MODBUS_REGISTERS["CURRENT"]]
        message.distance = registers[GRIPPER_MODBUS_REGISTERS["DISTANCE"]]

        self.gripper_status_publisher.publish(message)

    def process_drill_control_messages(self):
        if self.new_drill_control_message:
            self.drill_registers[DRILL_MODBUS_REGISTERS["DIRECTION"]] = self.drill_control_message.direction
            self.drill_registers[DRILL_MODBUS_REGISTERS["SPEED"]] = self.drill_control_message.speed

            self.drill_node.write_registers(0, self.drill_registers)
            self.modbus_nodes_seen_time = time()
            self.new_drill_control_message = False

    def gripper_control_message_received_callback(self, control_message):
        self.gripper_control_message = control_message
        self.new_gripper_control_message = True

    def science_control_message_received_callback(self, control_message):
        self.science_control_message = control_message
        self.new_science_control_message = True

    def drill_control_message_received_callback(self, control_message):
        self.drill_control_message = control_message
        self.new_drill_control_message = True

    def linear_control_message_received_callback(self, control_message):
        self.linear_control_message = control_message
        self.new_linear_control_message = True


def main(args=None):
    rclpy.init(args=args)
    effectors_control = EffectorsControl()
    rclpy.spin(effectors_control)
    effectors_control.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
