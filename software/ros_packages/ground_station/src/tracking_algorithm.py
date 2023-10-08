"""
Tracking algorithm code for the 2022-2023 Mars Rover RDF Capstone Project
Author: K. Kopcho
Date Revised: Jul 31st, 2023

"""

from geographiclib.geodesic import Geodesic
import gps
import math
import serial
import struct
import time
import socket

class TrackingAlgorithm:


#define Class variables below here
	r_lat = -1;
	r_lon = -1;
	b_lat = -1; 
	b_lon = -1;
	bearing = 0;
	sender_string = "" 
	base_fix = False
	base_fix_mode = ""
	rover_fix_mode = ""
    
#the following function is based on the equations found here: https://www.movable-type.co.uk/scripts/latlong.html
#also from this stack exchange thread https://gis.stackexchange.com/questions/29239/calculate-bearing-between-two-decimal-gps-coordinates/29240#29240
#and this example here https://web.archive.org/web/20171219235009/http://mathforum.org/library/drmath/view/55417.html
	def forward_bearing(self, base_lat, base_long, rover_lat, rover_long): 
		#convert to radians before doing calculations
		base_lat = math.radians(base_lat)
		base_long = math.radians(base_long)
		rover_lat = math.radians(rover_lat)
		rover_long = math.radians(rover_long)
		diff_long = rover_long - base_long
		#initial azimuth over great circle (higher accuracy?)
		
		dphi = math.log(math.tan(rover_lat/2.0+math.pi/4.0)/math.tan(base_lat/2.0+math.pi/4.0))
		if(abs(diff_long) > math.pi):
			if diff_long > 0.0:
				diff_long = -(2.0 * math.pi - diff_long)
			else:
				diff_long = (2.0 * math.pi + diff_long)
		theta1 = math.atan2(diff_long, dphi)
		bearing1 = (math.degrees(theta1) + 360) % 360
		print("Great circle bearing:", bearing1)

		#initial bearing using rhumb lines (the moveable type formula)
		y = math.sin(diff_long) * math.cos(rover_lat)
		x = math.cos(base_lat) * math.sin(rover_lat) - math.sin(base_lat) * math.cos(rover_lat) * math.cos(diff_long)
		theta2 = math.atan2(y,x)
		bearing2 = (math.degrees(theta1) + 360) % 360 #converts rads back to degrees from 0 to 360
		print("Rhumb line bearing:", bearing2)

		#using geodesic library for a sanity check...
		fbearing = Geodesic.WGS84.Inverse(base_lat, base_long, rover_lat, rover_long)['azi1']
		fbearing = round(fbearing, 1)
		print(fbearing)

		return fbearing

    	#below code adapted from the GPSD Example python client. Checks if lat/long of base are finite and then saves them as floats
	def base_read(self, session):
		if ((gps.isfinite(session.fix.latitude) and
			gps.isfinite(session.fix.longitude))): #makes sure we have a finite lat/lon
            
			#save lat/long as floats
			blat = session.fix.latitude
			blon = session.fix.longitude
		else:
			#set error values
			blat = -1
			blon = -1;

		return blat, blon

	def rover_read(self, port):
		line = port.readline() #reads line ended by '\n'
		line = str(line.decode()) #converts line bytes into a string literal
		if line == "": #if we've received nothing
			lat = -1
			long = -1
		else:
			coords = line.split(',') #splits line data into a multiple index list using a delimiter
			lat = float(coords[0]) #save first index as latitude
			long = float(coords[1]) #save last index as longitude
			time = coords[2]
			print("Rover GPS Time: %s" % time)

		return lat, long

	def run_algo(self, client):
			session = gps.gps(mode=gps.WATCH_ENABLE) #connect to the gps daemon
			port = serial.Serial('/dev/ttyACM1', timeout= 5) #open up the ACM USB port because it's where receiver is connected

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
								print(self.b_lat)
								print(self.b_lon)
								self.base_fix = True
				while self.base_fix is True:
						if(session.fix.mode == 0 | session.fix.mode == 1):
							self.base_fix_mode = "Base GPS Fix False"
							self.base_fix = False #re-get base fix
						else:
							self.base_fix_mode = "Base GPS Fix True"

						self.r_lat, self.r_lon = self.rover_read(port)
						#self.r_lat = round(self.r_lat, 6)
						#self.r_lon = round(self.r_lon, 6)
						print(self.r_lat)
						print(self.r_lon)

						if(self.r_lat != -1 and self.r_lon != -1):
								self.rover_fix_mode = "Rover GPS Fix True"
								self.bearing = self.forward_bearing(self.b_lat, self.b_lon, self.r_lat, self.r_lon)
								print("current bearing angle: %.1f degrees \n" % (self.bearing))
								bytes = str(self.bearing)
								print(bytes.encode())
								time.sleep(1)
								usb.write(bytes.encode())
								self.sender_string = str(self.b_lat) + ", " + str(self.b_lon) + ", " + str(self.r_lat) + ", " + str(self.r_lon) + ", " + str(self.bearing) + ", " + self.rover_fix_mode + ", " + self.base_fix_mode
								client.send(self.sender_string.encode())
						else:
								print("Cannot produce bearing angle, make sure GPS modules are getting a fix")
								self.rover_fix_mode = "Rover GPS Fix False"
								self.sender_string = "ERROR, " + self.rover_fix_mode + ", " + self.base_fix_mode
								client.send(self.sender_string.encode())
                    	

			except KeyboardInterrupt:
					print('')
            	

			# Got ^C, or fell out of the loop.  Cleanup, and leave.
			session.close()
			client.close()
			exit(0)


def main():
	tracking_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	addr = ('localhost', 5000)
	tracking_client.connect(addr)
	print("socket connected to:", addr)
	algo = TrackingAlgorithm()
	algo.run_algo(tracking_client) #pass sender into main running function so messages can be sent every cycle


if __name__ == "__main__":
    main()

