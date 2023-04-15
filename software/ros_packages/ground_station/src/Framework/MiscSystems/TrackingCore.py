from PyQt5 import QtWidgets, QtCore, QtGui, QtSerialPort, uic
import sys
import TrackingCoordinator as track
from functools import partial

class TrackingUI(QtWidgets.QWidget):
	def __init__(self):
		super(TrackingUI, self).__init__()
		uic.loadUi('TrackingAlgoDisp.ui', self)
		self.tThread = track.TrackingCore()
		self.tThread.rover_lat_update_ready__signal.connect(self.updateRLat)
		self.tThread.rover_lon_update_ready__signal.connect(self.updateRLon)
		self.tThread.base_lat_update_ready__signal.connect(self.updateBLat)
		self.tThread.base_lon_update_ready__signal.connect(self.updateBLon)
		self.tThread.bearing_update_ready__signal.connect(self.updateBearing)
		self.tThread.start()
		
		#only enable PB for manual angle on valid input in text box, default state is disabled
		self.manual_angle_pb.setEnabled(False)
		#set validator for angle text
		validator = QtGui.QDoubleValidator(0.00, 360.00, 2)
		
		#create serial object
		self.serial = QtSerialPort.QSerialPort('/dev/ttyUSB0')
		self.serial.setBaudRate(9600) #set baud to 9600
		self.serial.setDataBits(8) # data bits to 8
		self.serial.setStopBits(1) # 1 stop bit to match UART specs
		
		
		#connect signals and slots
		self.manual_angle_text.editingFinished.connect(partial(self.verify_angle, validator))
		self.manual_angle_pb.clicked.connect(self.send_angle)
	
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
		 

"""
Creating window is only necessary for standalone operation of the UI
app = QtWidgets.QApplication(sys.argv)
window = TrackingUI()
window.show()
app.exec_()
"""
