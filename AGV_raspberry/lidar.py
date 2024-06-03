import logging
import math
from time import sleep
from adafruit_rplidar import RPLidar, RPLidarException
import threading
from map import Map
from hex import HexRound, PolarToAxial
import serial.tools.list_ports

class Lidar:
    def __init__(self, slam):
        self.port = None
        self.lidar = None
        self.map = Map()
        self.res_scan = []
        self.slam = slam
        #in mm
        self.max_distance = 12000
        self.min_distance = 50
        self.hexHeight = 350
    
    def checkHealth(self):
        if self.lidar.health[1] == 0 : 
            return True
        else :
            return False

    def connect(self):
        try:
            if self.port is None:
                if not self.find_lidar_port():
                    raise Exception("Lidar not found")
            self.lidar = RPLidar(None, self.port, timeout=3)
            sleep(3)
            logging.info(f"Lidar connected : {self.lidar.info}") 
        except RPLidarException as e:
            logging.error(f"Lidar connection failed: {e}")
            sleep(5)
        except Exception as e:
            logging.error(f"Lidar connection failed: {e}")
            sleep(5)
    
    def find_lidar_port(self):
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            if p.pid == 60000:
                self.port = p.device
                return True
        return False

    def start(self):
        self.runThread = True
        self.thread = threading.Thread(target=self._scan, name="Lidar", daemon=True)
        self.thread.start()
    
    def _iter_scans(self, max_buf_meas: int = 500, min_len: int = 5):
        scan = []
        iterator = self.lidar.iter_measurements(max_buf_meas)
        for new_scan, quality, angle, distance in iterator:
            if new_scan:
                if len(scan) > min_len:
                    yield scan
                scan = []
            if quality > 0 and distance > self.min_distance:
                #rerotate based on lidar position
                scan.append((quality, (270 - angle) % 360, distance))

    def _scan(self):
        while True:
            if not self.runThread:
                break
            if self.lidar is None:
                self.connect()
                continue
            try:
                for scan in self._iter_scans():
                    if not self.runThread:
                        break
                    items = [item for item in scan]
                    distances = [item[2] for item in items]
                    angles = [item[1] for item in items]
                    self.slam.update(distances, angles)
                    self.res_scan = scan
                    self.convertToHex()
                    sleep(0.1)
            except RPLidarException as e:
                logging.error(f"Lidar error: {e}")
                self.lidar.reset()
                sleep(5)

    def convertToHex(self):
        self.map.clearMap()
        for _,angle, distance in self.res_scan:
            p, q, r = PolarToAxial(distance, angle, self.hexHeight)
            p, q, r = HexRound(p, q, r)
            self.map.addObstacle(p, q)

    def getLocalMap(self):
        obstacles = self.map.getObstacles()
        for i in range(len(obstacles)):
            obstacles[i] = {'x': obstacles[i].q, 'y': obstacles[i].r}
        return obstacles
    
    def getPos(self):
        pose = self.slam.getPos()
        orientation = pose[2]
        if orientation > 0:
            orientation += 90
            orientation = orientation % 360
        else:
            orientation = (90 + orientation) * -1
            orientation = orientation % 360
            orientation = 360 - orientation
        lst = list(pose)
        lst[0] *= -1
        lst[1] *= -1
        lst[0] -= 5000
        lst[1] -= 5000
        lst[2] = orientation
        pose = tuple(lst)
        return pose

    def getFront(self):
        total = 0
        count = 0
        for i in range(80, 100):
            if self.res_scan[i] == 0:
                continue
            total += self.res_scan[i]
            count += 1
        if total == 0:
            return 0
        return total / count

    def getBack(self):
        total = 0
        count = 0
        for i in range(260, 280):
            if self.res_scan[i] == 0:
                continue
            total += self.res_scan[i]
            count += 1
        if total == 0:
            return 0
        return total / count
        
    def stop(self):
        try:
            self.runThread = False
            self.thread.join()
            if not (self.lidar is None):
                self.lidar.stop()
                self.lidar.disconnect() 
        except Exception as e:
            logging.error(f"Lidar Error: {e}")

