from PyQt5 import QtWidgets, QtCore, QtSerialPort, QtGui
import random
import logging
from time import time
from time import sleep
from functools import partial
from multiprocessing.connection import Listener

#Dearborn Hall coordinates: 44.56688122224506, -123.27560553741544
#Near Merryfield coordinates: 44.566890589052235, -123.27462028171236

THREAD_HERTZ = 5
left = "onescreen" #left_screen

#create threaded class to avoid blocking UI updates
class TrackingCore(QtCore.QThread):
	#create signals
	rover_lat_update_ready__signal = QtCore.pyqtSignal(float)
	rover_lon_update_ready__signal = QtCore.pyqtSignal(float)
	base_lat_update_ready__signal = QtCore.pyqtSignal(float)
	base_lon_update_ready__signal = QtCore.pyqtSignal(float)
	bearing_update_ready__signal = QtCore.pyqtSignal(float)

	def __init__(self,shared_objects):
		super(TrackingCore,self).__init__()
		
		# ########## Reference to class init variables ##########
		self.shared_objects = shared_objects
		self.left_screen = self.shared_objects["screens"][left]
		
		self.rover_lat = self.left_screen.rover_lat #type: QtWidgets.QLabel
		self.rover_lon = self.left_screen.rover_lon #type: QtWidgets.QLabel
		self.base_lat = self.left_screen.base_lat #type: QtWidgets.QLabel
		self.base_lon = self.left_screen.base_lon #type: QtWidget.QLabel
		self.manual_angle_text = self.left_screen.manual_angle_text #type: QtWidgets.QLineEdit
		self.manual_angle_pb = self.left_screen.manual_angle_pb #type: QtWidgets.QPushButton
		
		# ########## Get the settings instance ##########
		self.settings = QtCore.QSettings()
        	
        	# ########## Thread Flags ##########
		self.run_thread_flag = True
		
		# ########## Get the Pick And Plate instance of the logger ##########
		self.logger = logging.getLogger("groundstation")
		
		# ########## Class Variables ##########
		self.wait_time = 1.0 / THREAD_HERTZ
		
		self.current_rover_lat = -1
		self.current_rover_lon = -1
		self.current_base_lat = -1
		self.current_base_lon = -1
		self.current_bearing = -1
		
		#only enable PB for manual angle on valid input in text box, default state is disabled
		self.manual_angle_pb.setEnabled(False)
		
		#create serial object
		self.serial = QtSerialPort.QSerialPort('/dev/ttyUSB0')
		self.serial.setBaudRate(9600) #set baud to 9600
		self.serial.setDataBits(8) # data bits to 8
		self.serial.setStopBits(1) # 1 stop bit to match UART specs
		
		
	def run(self):
		self.logger.debug("Starting Tracking Thread")
		
		addr = ('localhost', 5000)
		ui_listener = Listener(addr)
		conn = ui_listener.accept() #accept conn from tracking algo
		msg = ""
		while self.run_thread_flag:
			start_time = time()
			if conn.poll():
				msg = conn.recv()
				print(f"got message: {msg}")
				if(msg == "ERROR"):
					print("Tracking algorithm encountered an error")
					continue
				else:
					self.tracking_updates_callback(msg)
		
		
		conn.close()
		time_diff = time() - start_time
		self.msleep(max(int(self.wait_time - time_diff), 0))

		self.logger.debug("Stopping Tracking Thread")

            
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
		
	def updateRLat(self, value):
		self.rover_lat.setNum(value)
    	
	def updateRLon(self, value):
		self.rover_lon.setNum(value)
		
	def updateBLat(self, value):
		self.base_lat.setNum(value)
		
	def updateBLon(self, value):
		self.base_lon.setNum(value)
		
	def updateBearing(self,value):
		self.bearing_angle.setNum(value)
		
	def verify_angle(self, validator):
		state = validator.validate(self.manual_angle_text.text(), 0)
		if(state[0] == QtGui.QValidator.Acceptable):
			self.manual_angle_pb.setEnabled(True)
		if(state[0] == QtGui.QValidator.Invalid or state[0] == QtGui.QValidator.Intermediate):
			self.manual_angle_pb.setEnabled(False)
			
	def send_angle(self):
		angle = self.manual_angle_text.text()
		self.serial.open(QtCore.QIODevice.WriteOnly)
		self.serial.write(angle.encode())
		
	def connect_signals_and_slots(self):
		self.rover_lat_update_ready__signal.connect(self.updateRLat)
		self.rover_lon_update_ready__signal.connect(self.updateRLon)
		self.base_lat_update_ready__signal.connect(self.updateBLat)
		self.base_lon_update_ready__signal.connect(self.updateBLon)
		self.bearing_update_ready__signal.connect(self.updateBearing)
		self.manual_angle_pb.clicked.connect(self.send_angle)
		
		#set validator for angle text
		validator = QtGui.QDoubleValidator(0.00, 360.00, 2)
		self.manual_angle_text.editingFinished.connect(partial(self.verify_angle, validator))
		
	def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
		start_signal.connect(self.start)
		signals_and_slots_signal.connect(self.connect_signals_and_slots)
		kill_signal.connect(self.on_kill_threads_requested__slot)		
		
	def on_kill_threads_requested__slot(self):
		self.run_thread_flag = False
			
