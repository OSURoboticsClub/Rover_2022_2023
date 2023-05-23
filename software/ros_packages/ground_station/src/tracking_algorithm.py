"""
Tracking algorithm code for the 2022-2023 Mars Rover RDF Capstone Project
Author: K. Kopcho
Date Revised: 03/11/2023

"""

import gps
import math
import serial
import struct
import time
from multiprocessing.connection import Client

class TrackingAlgorithm:


#define Class variables below here
	r_lat = -1;
	r_lon = -1;
	b_lat = -1; 
	b_lon = -1;
	bearing = 0;
	sender_string = "" 
	base_fix = False
    
#the following function is based on the equations found here: https://www.movable-type.co.uk/scripts/latlong.html
	def forward_bearing(self, base_lat, base_long, rover_lat, rover_long): 
		diff_long = (rover_long - base_long)

		#convert to radians before doing calculations
		base_lat = math.radians(base_lat)
		base_long = math.radians(base_long)
		rover_lat = math.radians(rover_lat)
		rover_long = math.radians(rover_long)
		diff_long = math.radians(diff_long)

        	#do the actual math now, again this is based on the equations on the moveable-type site
		y = math.sin(diff_long) * math.cos(rover_lat)
		x = math.cos(base_lat) * math.sin(rover_lat) - math.sin(base_lat) * math.cos(rover_lat) * math.cos(diff_long)
		theta = math.atan2(y, x)
		fbearing = (theta * 180/math.pi + 360) % 360 #converts rads back to degrees from 0 to 360
		fbearing = round(fbearing, 1)

		return fbearing

    	#below code adapted from the GPSD Example python client. Checks if lat/long of base are finite and then saves them as floats
	def base_read(self, session):
		if ((gps.isfinite(session.fix.latitude) and
			gps.isfinite(session.fix.longitude))): #makes sure we have a finite lat/lon
            
			#save lat/long as floats
			blat = session.fix.latitude
			blon = session.fix.longitude
			blat = round(blat, 2)
			blon = round(blon, 2)
		else:
			#set error values
			blat = -1
			blon = -1;

		return blat, blon

	def rover_read(self, port):
		line = port.readline() #reads line ended by '\n'
		line = str(line.decode()) #converts line bytes into a string literal
		coords = line.split(',') #splits line data into a multiple index list using a delimiter
		lat = float(coords[0]) #save first index as latitude
		long = float(coords[1]) #save last index as longitude
		time = coords[2]
		print("Rover GPS Time: %s" % time)

		return lat, long

	def run_algo(self, client):
			session = gps.gps(mode=gps.WATCH_ENABLE) #connect to the gps daemon
			port = serial.Serial('/dev/ttyACM1') #open up the ACM USB port because it's where the Feather is connected

			#initialize parameters for USB serial to motor controller
			usb = serial.Serial('/dev/ttyUSB0') #open up USB port for serial comms to turntable controller
			#configure for UART protocol
			usb.baudrate = 9600 #9600 baud
			usb.bytesize = serial.EIGHTBITS #8 bit data
			usb.stopbit = serial.STOPBITS_ONE
			usb.parity = serial.PARITY_NONE

			try:
				while self.base_fix is False:
						if session.read() == 0:
								if not (gps.MODE_SET & session.valid): #ensure we have a TPV Packet from read
									continue
						print("Base GPS Time %s" % session.fix.time)

						self.b_lat, self.b_lon = self.base_read(session) #get values from the base gps
						if(self.b_lat == -1 and self.b_lon == -1): #if we're not connected or lat/lon isn't finite
								print(" Lat n/a Lon n/a")
						else:
								print("base lat: %.6f, base lon: %.6f \n" % (self.b_lat, self.b_lon))
								self.base_fix = True
				while self.base_fix is True:
						self.r_lat, self.r_lon = self.rover_read(port)
						#self.r_lat = round(self.r_lat, 3)+
						#self.r_lon = round(self.r_lon, 3)
						print("rover lat: %.6f, rover long %.6f \n" % (self.r_lat, self.r_lon))

						if(self.r_lat != -1 and self.r_lon != -1):
								self.bearing = self.forward_bearing(self.b_lat, self.b_lon, self.r_lat, self.r_lon)
								print("current bearing angle: %.1f degrees \n" % (self.bearing))
								bytes = str(self.bearing)
								print(bytes.encode())
								time.sleep(1)
								usb.write(bytes.encode())
								self.sender_string = str(self.b_lat) + ", " + str(self.b_lon) + ", " + str(self.r_lat) + ", " + str(self.r_lon) + ", " + str(self.bearing)
								client.send(self.sender_string)
						else:
								print("Cannot produce bearing angle, make sure GPS modules are getting a fix")
								client.send("ERROR")
                    			

			except KeyboardInterrupt:
					print('')
            	

			# Got ^C, or fell out of the loop.  Cleanup, and leave.
			session.close()
			client.close()
			exit(0)


def main():
	addr = ('localhost', 5000) #open a socket at addr 5000
	tracking_client = Client(addr)
	algo = TrackingAlgorithm()
	algo.run_algo(tracking_client) #pass sender into main running function so messages can be sent every cycle


if __name__ == "__main__":
    main()

