#!/usr/bin/env python
#####################################
# Imports
#####################################
# Python native imports
import rclpy
import serial
from time import time
import json
import re

from rclpy.node import Node

from nmea_msgs.msg import Sentence
from sensor_msgs.msg import Imu

#####################################
# Global Variables
#####################################
NODE_NAME = "tower_odometry"

DEFAULT_PORT = "/dev/rover/ttyOdometry"
# DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 115200


# DEFAULT_GPS_SENTENCE_TOPIC = "gps/sentence"
DEFAULT_GPS_SENTENCE_TOPIC = "gps/sentence"
# DEFAULT_IMU_TOPIC = "imu/data"

DEFAULT_HERTZ = 100

ODOM_LAST_SEEN_TIMEOUT = 1  # seconds


#####################################
# Odometry Class Definition
#####################################
class Odometry(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        self.port = self.declare_parameter("~port", DEFAULT_PORT).value
        self.baud = self.declare_parameter("~baud", DEFAULT_BAUD).value

        self.gps_sentence_topic = self.declare_parameter('~gps_sentence_topic', DEFAULT_GPS_SENTENCE_TOPIC).value
        #self.imu_data_topic = self.declare_parameter("~imu_data_topic", DEFAULT_IMU_TOPIC).value

        self.wait_time = 1.0 / self.declare_parameter('~hertz', DEFAULT_HERTZ).value

        self.odom_serial = serial.Serial(port=self.port, baudrate=self.baud)
        self.odom_serial.setRTS(0)

        self.sentence_publisher = self.create_publisher(Sentence, self.gps_sentence_topic, 1)
        #self.imu_data_publisher = self.create_publisher(Imu, self.imu_data_topic, queue_size=1)

        self.odom_last_seen_time = time()

        self.timer = self.create_timer(self.wait_time, self.process_messages)

    def process_messages(self):
        if self.odom_serial.in_waiting > 0:
            line = self.odom_serial.readline()

            temp = json.loads(line)

            gps = temp.get('gps', None)
            #imu = temp.get('imu', None)
            #imu_cal = temp.get('imu_cal', None)

            if gps:
                # ###### THIS IS HERE TO DEAL WITH UBLOX GPS #####
                if "GNGGA" in gps:
                    gps = gps.replace("GNGGA", "GPGGA")
                    gps = gps[:-2] + str(self.chksum_nmea(gps))[2:]
                # print(gps)
                # #####

                self.broadcast_gps(gps)

            #if imu:
                # print(imu)
            #    self.broadcast_imu(imu)

            # if imu_cal:
            #     print(imu_cal)

            self.odom_last_seen_time = time()

        if (time() - self.odom_last_seen_time) > ODOM_LAST_SEEN_TIMEOUT:
            print ("Odometry not seen for", ODOM_LAST_SEEN_TIMEOUT, "seconds. Exiting.")
            self.destroy_node()
            return  # Exit so respawn can take over

    @staticmethod
    def chksum_nmea(sentence):
        # String slicing: Grabs all the characters
        # between '$' and '*' and nukes any lingering
        # newline or CRLF
        chksumdata = re.sub("(\n|\r\n)", "", sentence[sentence.find("$") + 1:sentence.find("*")])

        # Initializing our first XOR value
        csum = 0

        # For each char in chksumdata, XOR against the previous
        # XOR'd char.  The final XOR of the last char will be our
        # checksum to verify against the checksum we sliced off
        # the NMEA sentence

        for c in chksumdata:
            # XOR'ing value of csum against the next char in line
            # and storing the new XOR value in csum
            csum ^= ord(c)

        # Do we have a validated sentence?
        return hex(csum)

    def broadcast_gps(self, gps):
        message = Sentence()
        message.header.frame_id = "gps"
        message.header.stamp = self.get_clock().now()
        message.sentence = gps
        self.sentence_publisher.publish(message)

    def broadcast_imu(self, imu):
        message = Imu()
        message.header.frame_id = "imu"
        message.header.stamp = self.get_clock().now()

        message.orientation.x = imu["ox"]
        message.orientation.y = imu["oy"]
        message.orientation.z = imu["oz"]
        message.orientation.w = imu["ow"]

        message.angular_velocity.x = imu["avx"]
        message.angular_velocity.y = imu["avy"]
        message.angular_velocity.z = imu["avz"]

        message.linear_acceleration.x = imu["lax"]
        message.linear_acceleration.y = imu["lay"]
        message.linear_acceleration.z = imu["laz"]

        self.imu_data_publisher.publish(message)

def main(args=None):
	rclpy.init(args=args)
	odometry = Odometry()
	rclpy.spin(odometry)
	odometry.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
    main()
