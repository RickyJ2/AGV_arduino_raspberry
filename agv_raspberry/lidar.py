import logging
import math
from time import sleep
from adafruit_rplidar import RPLidar, RPLidarException
import threading

class Lidar:
    def __init__(self, port = '/dev/ttyUSB0'):
        self.port = port
        self.scan_data = [0] * 360
        self.lidar = None
        #in mm
        self.max_distance = 10000
        self.min_distance = 0
    
    def checkHealth(self):
        if self.lidar.health[1] == 0 : 
            return True
        else :
            return False

    def connect(self):
        try:
            self.lidar = RPLidar(None, self.port, timeout=3)
            sleep(3)
            logging.info(f"Lidar connected : {self.lidar.info}") 
        except RPLidarException as e:
            logging.error(f"Lidar connection failed: {e}")
            sleep(5)
        
    def start(self):
        self.runThread = True
        self.thread = threading.Thread(target=self._scan, daemon=True)
        self.thread.start()

    def _scan(self):
        while True:
            if self.lidar is None:
                self.connect()
                continue
            if not self.runThread:
                break
            try:
                for scan in self.lidar.iter_scans():
                    if not self.runThread:
                        break
                    temp = [0]*360
                    for _, angle, distance in scan:
                        if distance > self.max_distance or distance < self.min_distance:
                            temp[min([359, math.floor(angle)])] = 0
                            continue
                        temp[min([359, math.floor(angle)])] = distance
                    self.scan_data = temp
            except RPLidarException as e:
                logging.error(f"Lidar error: {e}")
                self.lidar.reset()
                sleep(5)

    def getScanData(self):
        return self.scan_data

    def stop(self):
        try:
            self.runThread = False
            self.thread.join()
            if not (self.lidar is None):
                self.lidar.stop()
                self.lidar.disconnect() 
        except Exception as e:
            logging.error(f"Lidar Error: {e}")