import serial
import time

truePacket = [0x01, 0x10, 0x03, 0x00, 0x03, 0x01, 0x01, 0x01, 0x01, 0xe1]
falsePacket = [0x01, 0x10, 0x03, 0x00, 0x03, 0x01, 0x01, 0x00, 0xc0, 0x21]

node = serial.Serial('COM9',500000, timeout=0.01)

while(1):
    node.write(bytes(truePacket))
    time.sleep(2)
    node.write(bytes(falsePacket))
    time.sleep(2)