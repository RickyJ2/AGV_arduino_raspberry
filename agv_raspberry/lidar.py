import logging
import math
from time import sleep
from adafruit_rplidar import RPLidar, RPLidarException
import threading
from map import Map
from hex import cubeRound

class Lidar:
    def __init__(self, arduino, port = '/dev/ttyUSB1'):
        self.port = port
        self.lidar = None
        self.map = Map()
        self.res_scan = [0] * 360
        #in mm
        self.max_distance = 5000
        self.min_distance = 0
        self.arduino = arduino
    
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
        
    def start(self, scan):
        self.runThread = True
        self.thread = threading.Thread(target=scan, name="Lidar", daemon=True)
        self.thread.start()

    def convertToHex(self):
        self.map.clearMap()
        for i, distance in enumerate(self.res_scan):
            if distance == 0:
                continue
            angle = math.radians(360-i) #need convertion for this formula works
            hexHeight = 350 #in mm
            size = hexHeight/2
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            q = (math.sqrt(3)/3 * x  - 1.0/3 * y) / size
            r = (2.0/3 * y) / size
            q, r = cubeRound(q, r)
            # print(i, math.floor(distance), q, r)
            self.map.addObstacle(q, r)
        # print(self.getLocalMap())

    def getLocalMap(self):
        obstacles = self.map.getObstacles()
        for i in range(len(obstacles)):
            obstacles[i] = {'x': obstacles[i].q, 'y': obstacles[i].r}
        return obstacles
        
    def stop(self):
        try:
            self.runThread = False
            self.thread.join()
            if not (self.lidar is None):
                self.lidar.stop()
                self.lidar.disconnect() 
        except Exception as e:
            logging.error(f"Lidar Error: {e}")
