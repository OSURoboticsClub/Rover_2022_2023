from PyQt5 import QtWidgets, QtCore, QtGui
import sys
import random
import time
from time import sleep
import logging
from multiprocessing.connection import Listener

THREAD_HERTZ = 5

class TrackingCallback(QtCore.QThread):
    #create signals
    rover_lat_update_ready__signal = QtCore.pyqtSignal(float)
    rover_lon_update_ready__signal = QtCore.pyqtSignal(float)
    base_lat_update_ready__signal = QtCore.pyqtSignal(float)
    base_lon_update_ready__signal = QtCore.pyqtSignal(float)
    bearing_update_ready__signal = QtCore.pyqtSignal(float)

    def __init__(self):
        super(TrackingCallback, self).__init__()
        
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


    def connect_signals_and_slots(self):
        pass

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)		
    
    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False
                    
	

