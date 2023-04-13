from PyQt5 import QtWidgets, QtCore, QtGui
import sys
import random
from time import sleep
from multiprocessing.connection import Listener

#Dearborn Hall coordinates: 44.56688122224506, -123.27560553741544
#Near Merryfield coordinates: 44.566890589052235, -123.27462028171236

#create threaded class to avoid blocking UI updates
class TrackingCore(QtCore.QThread):
	print("Tracking Thread Started!")
	#create signals
	rover_lat_update_ready__signal = QtCore.pyqtSignal(float)
	rover_lon_update_ready__signal = QtCore.pyqtSignal(float)
	base_lat_update_ready__signal = QtCore.pyqtSignal(float)
	base_lon_update_ready__signal = QtCore.pyqtSignal(float)
	bearing_update_ready__signal = QtCore.pyqtSignal(float)

	def __init__(self):
		super(TrackingCore,self).__init__()
		## Class Variables ##
		self.current_rover_lat = -1
		self.current_rover_lon = -1
		self.current_base_lat = -1
		self.current_base_lon = -1
		self.current_bearing = -1
		
		
	def tracking_updates_callback(self, str):
		updates = str.split(',')
		self.current_base_lat = float(updates[0])
		self.current_base_lon = float(updates[1])
		self.current_rover_lat = float(updates[2])
		self.current_rover_lon = float(updates[3])
		self.current_bearing = float(updates[4])
		
		self.base_lat_update_ready__signal.emit(self.current_base_lat)
		self.base_lon_update_ready__signal.emit(self.current_base_lon)
		self.rover_lat_update_ready__signal.emit(self.current_rover_lat)
		self.rover_lon_update_ready__signal.emit(self.current_rover_lon)
		self.bearing_update_ready__signal.emit(self.current_bearing)
	
	"""
	#Verification functions below here	
	def get_random_coordinate(self, start_val, stop_val):
		coord = random.uniform(start_val, stop_val)
		return coord
	
	def rover_latitude_changed__slot(self):
		#creating false coordinates for verification purposes, actual final code will take in input from tracking algo
		self.current_rover_lat = self.get_random_coordinate(42.56688, 48.2333)
		self.rover_lat_update_ready__signal.emit(self.current_rover_lat)
		sleep(0.2)

	def rover_longitude_changed__slot(self):
		self.current_rover_lon = self.get_random_coordinate(-120.27560, -125.3241)
		self.rover_lon_update_ready__signal.emit(self.current_rover_lon)
		sleep(0.2)
	
	def base_latitude_changed__slot(self):
		self.current_base_lat = self.get_random_coordinate(40.5668905, 46.323212)
		self.base_lat_update_ready__signal.emit(self.current_base_lat)
		sleep(0.2)
		

	def base_longitude_changed__slot(self):
		self.current_base_lon = self.get_random_coordinate(-119.274620, -125.274620)
		self.base_lon_update_ready__signal.emit(self.current_base_lon)
		sleep(0.2)
	"""
		
	def run(self):
		addr = ('localhost', 5000)
		ui_listener = Listener(addr)
		conn = ui_listener.accept() #accept conn from tracking algo
		msg = ""
		while 1:
			if conn.poll():
				msg = conn.recv()
				print(f"got message: {msg}")
				if(msg == "ERROR"):
					print("Tracking algorithm encountered an error")
					continue
				else:
					self.tracking_updates_callback(msg)
		conn.close()
			
