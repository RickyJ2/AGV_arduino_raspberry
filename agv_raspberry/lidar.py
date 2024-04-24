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
        
    def start(self):
        self.runThread = True
        self.thread = threading.Thread(target=self._scan, name="Lidar", daemon=True)
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
                    temp = [0] * 360
                    for _, angle, distance in scan:
                        ang = (270 - (math.floor(angle))) % 360 
                        if distance > self.max_distance: 
                            temp[ang] = self.max_distance
                            continue
                        elif distance < self.min_distance:
                            temp[ang] = 0
                            continue
                        temp[ang] = distance
                    self.res_scan = temp
                    self.convertToHex()
            except RPLidarException as e:
                logging.error(f"Lidar error: {e}")
                self.lidar.reset()
                sleep(5)

    def convertToHex(self):
        self.map.clearMap()
        for i, distance in enumerate(self.res_scan):
            if distance == 0:
                continue
            ang = i
            robotOrientation = 90 - self.arduino.getOrientation()
            if robotOrientation < 90:
                ang = (360 + ang - robotOrientation) % 360
            else:
                ang = (ang + robotOrientation) % 360
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

