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

# Custom Imports
from rover2_control_interface.msg import DriveControlMessage, DriveStatusMessage

#####################################
# Global Variables
#####################################
NODE_NAME = "drive_control"

DEFAULT_PORT = "/dev/rover/ttyBogie"
DEFAULT_BAUD = 115200

DEFAULT_INVERT = False

DEFAULT_DRIVE_CONTROL_TOPIC = "drive_control/rear"
DEFAULT_DRIVE_CONTROL_STATUS_TOPIC = "drive_status/rear"

FIRST_MOTOR_ID = 1
SECOND_MOTOR_ID = 2

COMMUNICATIONS_TIMEOUT = 0.01  # Seconds

RX_DELAY = 0.01
TX_DELAY = 0.01

DEFAULT_HERTZ = 30

MODBUS_REGISTERS = {
    "DIRECTION": 0,
    "SPEED": 1,
    "SLEEP": 2,

    "CURRENT": 3,
    "FAULT": 4,

    "TEMPERATURE": 5
}

MOTOR_DRIVER_DEFAULT_MESSAGE = [
    1,  # Forwards
    0,  # 0 Speed
    1  # Not in sleep mode
]

UINT16_MAX = 65535

BOGIE_LAST_SEEN_TIMEOUT = 2  # seconds


#####################################
# DriveControl Class Definition
#####################################
class DriveControl(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        self.port = self.get_parameter_or("~port", DEFAULT_PORT)
        self.baud = self.get_parameter_or("~baud", DEFAULT_BAUD)

        print(self.port)

        self.first_motor_id = self.get_parameter_or("~first_motor_id", FIRST_MOTOR_ID)
        self.second_motor_id = self.get_parameter_or("~second_motor_id", SECOND_MOTOR_ID)

        self.first_motor_inverted = self.get_parameter_or("~invert_first_motor", DEFAULT_INVERT)
        self.second_motor_inverted = self.get_parameter_or("~invert_second_motor", DEFAULT_INVERT)

        self.drive_control_subscriber_topic = self.get_parameter_or("~drive_control_topic", DEFAULT_DRIVE_CONTROL_TOPIC)

        self.drive_control_status_topic = self.get_parameter_or("~drive_control_status_topic", DEFAULT_DRIVE_CONTROL_STATUS_TOPIC)

        self.wait_time = 1.0 / self.get_parameter_or("~hertz", DEFAULT_HERTZ)

        self.first_motor = None
        self.second_motor = None

        self.connect_to_bogie()

        self.drive_control_subscriber = \
            self.create_subscription(DriveControlMessage, self.drive_control_subscriber_topic, self.drive_control_callback, 10)

        self.drive_control_status_publisher = self.create_publisher(DriveStatusMessage, self.drive_control_status_topic, 1)

        self.drive_control_message = DriveControlMessage()
        self.new_control_message = False

        self.bogie_last_seen = time()

        self.run()

    def __setup_minimalmodbus_for_485(self):
        self.first_motor.serial = serial.rs485.RS485(self.port, baudrate=self.baud, timeout=COMMUNICATIONS_TIMEOUT)
        self.first_motor.serial.rs485_mode = serial.rs485.RS485Settings(rts_level_for_rx=1, rts_level_for_tx=0,
                                                                        delay_before_rx=RX_DELAY,
                                                                        delay_before_tx=TX_DELAY)

        self.second_motor.serial = serial.rs485.RS485(self.port, baudrate=self.baud, timeout=COMMUNICATIONS_TIMEOUT)
        self.second_motor.serial.rs485_mode = serial.rs485.RS485Settings(rts_level_for_rx=1, rts_level_for_tx=0,
                                                                         delay_before_rx=RX_DELAY,
                                                                         delay_before_tx=TX_DELAY)

    def run(self):
        # Note: perhaps an Executor is needed
        # https://docs.ros2.org/foxy/api/rclpy/api/execution_and_callbacks.html#rclpy.executors.Executor
        while rclpy.ok():
            start_time = time()

            try:
                self.send_drive_control_message()
                self.get_drive_status()

            except Exception as error:
                pass

            if (time() - self.bogie_last_seen) > BOGIE_LAST_SEEN_TIMEOUT:
                print(f"Bogie not seen for {BOGIE_LAST_SEEN_TIMEOUT} seconds. Exiting.")
                return  # Exit so respawn can take over

            time_diff = time() - start_time

            sleep(max(self.wait_time - time_diff, 0))

    def connect_to_bogie(self):
        self.first_motor = minimalmodbus.Instrument(self.port, int(self.first_motor_id))
        self.second_motor = minimalmodbus.Instrument(self.port, int(self.second_motor_id))
        self.__setup_minimalmodbus_for_485()

    def send_drive_control_message(self):
        if self.new_control_message:
            drive_control = self.drive_control_message
            try:
                first_motor_register_data = list(MOTOR_DRIVER_DEFAULT_MESSAGE)
                first_direction = \
                    not drive_control.first_motor_direction if self.first_motor_inverted else drive_control.first_motor_direction
                first_motor_register_data[MODBUS_REGISTERS["DIRECTION"]] = first_direction
                first_motor_register_data[MODBUS_REGISTERS["SPEED"]] = min(drive_control.first_motor_speed, UINT16_MAX)

                second_motor_register_data = list(MOTOR_DRIVER_DEFAULT_MESSAGE)
                second_direction = not drive_control.second_motor_direction if self.second_motor_inverted else drive_control.second_motor_direction
                second_motor_register_data[MODBUS_REGISTERS["DIRECTION"]] = second_direction
                second_motor_register_data[MODBUS_REGISTERS["SPEED"]] = min(drive_control.second_motor_speed, UINT16_MAX)

                self.first_motor.write_registers(0, first_motor_register_data)
                self.second_motor.write_registers(0, second_motor_register_data)

            except Exception as error:
                pass

            self.new_control_message = False

    def get_drive_status(self):
        status = DriveStatusMessage()

        first_motor_status = [0, 0, 0]
        second_motor_status = [0, 0, 0]

        try:
            first_motor_status = self.first_motor.read_registers(3, 3)
            status.first_motor_connected = True
        except Exception:
            status.first_motor_connected = False

        try:
            second_motor_status = self.second_motor.read_registers(3, 3)
            status.second_motor_connected = True
        except Exception:
            status.second_motor_connected = False

        if status.first_motor_connected or status.second_motor_connected:
            self.bogie_last_seen = time()

        if status.first_motor_connected:
            status.first_motor_current = first_motor_status[0] / 1000.0
            status.first_motor_fault = first_motor_status[1]
            status.first_motor_temp = first_motor_status[2] / 1000.0

        if status.second_motor_connected:
            status.second_motor_current = second_motor_status[0] / 1000.0
            status.second_motor_fault = second_motor_status[1]
            status.second_motor_temp = second_motor_status[2] / 1000.0

        self.drive_control_status_publisher.publish(status)

    def drive_control_callback(self, drive_control):
        self.drive_control_message = drive_control
        self.new_control_message = True


def main(args=None):
    rclpy.init(args=args)
    drive_control = DriveControl()
    rclpy.spin(drive_control)
    drive_control.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()