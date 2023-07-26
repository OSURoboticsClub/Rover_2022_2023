from PyQt5 import QtWidgets, QtCore, QtGui
import sys
import random
from time import time
from time import sleep
import logging
import socket

THREAD_HERTZ = 5

class TrackingCallback(QtCore.QThread):
    #create signals
    rover_lat_update_ready__signal = QtCore.pyqtSignal(float)
    rover_lon_update_ready__signal = QtCore.pyqtSignal(float)
    base_lat_update_ready__signal = QtCore.pyqtSignal(float)
    base_lon_update_ready__signal = QtCore.pyqtSignal(float)
    bearing_update_ready__signal = QtCore.pyqtSignal(float)
    base_fix_update_ready__signal = QtCore.pyqtSignal(str)
    rover_fix_update_ready__signal = QtCore.pyqtSignal(str)

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
        self.current_base_status = "Base GPS Fix False"
        self.current_rover_status = "Rover GPS Fix False"


    def run(self):
        self.logger.debug("Starting Tracking Thread")
        #create listener socket and bind to localhost for IPC (yes I know better ways exist...)
        ui_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        addr = ('localhost', 5000)
        ui_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) #make socket reusable
        ui_sock.bind(addr)

        ui_sock.settimeout(15) #make sure the socket does not block on recv so UI does not freeze
        #ref: https://stackoverflow.com/questions/16745409/what-does-pythons-socket-recv-return-for-non-blocking-sockets-if-no-data-is-r
        ui_sock.listen(1)
        conn, addr = ui_sock.accept()
        msg = ""
        with conn:
            while self.run_thread_flag:
                start_time = time()
                try:
                    msg = conn.recv(4096)
                except socket.timeout as e:
                   print("Timeout on recv, try again later")
                   continue
                except socket.error as e:
                    print("some other error occured")
                    print(e)
                    sys.exit(1)
                else:
                    #print(f"got message: {msg}")
                    msg = msg.decode()
                    if(msg[0:5] == "ERROR"):
                        print("Tracking algorithm encountered an error")
                        status = msg.split(",")
                        self.current_rover_status = status[1]
                        self.current_base_status = status[2]
                        self.base_fix_update_ready__signal.emit(self.current_base_status)
                        self.rover_fix_update_ready__signal.emit(self.current_rover_status)
                        continue
                    else:
                        self.tracking_updates_callback(msg)

        conn.close()
        time_diff = time() - start_time
        self.msleep(max(int(self.wait_time - time_diff), 0))
        self.logger.debug("Stopping Tracking Thread")


    def tracking_updates_callback(self, str):
        updates = str.split(b',')
        self.current_base_lat = float(updates[0])
        self.current_base_lon = float(updates[1])
        self.current_rover_lat = float(updates[2])
        self.current_rover_lon = float(updates[3])
        self.current_bearing = float(updates[4])
        self.current_rover_status = updates[5].decode()
        self.current_base_status = updates[6].decode()

        self.base_lat_update_ready__signal.emit(self.current_base_lat)
        self.base_lon_update_ready__signal.emit(self.current_base_lon)
        self.rover_lat_update_ready__signal.emit(self.current_rover_lat)
        self.rover_lon_update_ready__signal.emit(self.current_rover_lon)
        self.bearing_update_ready__signal.emit(self.current_bearing)
        self.base_fix_update_ready__signal.emit(self.current_base_status)
        self.rover_fix_update_ready__signal.emit(self.current_rover_status)


    def connect_signals_and_slots(self):
        pass

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)		
    
    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False
                    
	

