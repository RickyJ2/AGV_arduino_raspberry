import logging
from time import sleep
from adafruit_rplidar import RPLidar
import threading
import serial.tools.list_ports
from Class.slam import SLAM

class Lidar:
    def __init__(self, slam: SLAM):
        self.port = None
        self.lidar: RPLidar = None
        self.slam: SLAM = slam
        #in mm
        self.max_distance = 12000
        self.min_distance = 50
        self.scan: list[tuple] = []
        self.corners: list[tuple] = []
    
    def checkHealth(self) -> bool:
        if self.lidar.health[1] == 0: 
            return True
        else:
            return False

    def connect(self):
        try:
            if self.port is None:
                if not self.find_lidar_port():
                    raise Exception("Lidar not found")
            self.lidar = RPLidar(None, self.port, timeout=3)
            sleep(3)
            logging.info(f"Lidar connected : {self.lidar.info}") 
        except Exception as e:
            logging.error(f"Lidar connection failed: {e}")
            sleep(5)
    
    def find_lidar_port(self) -> bool:
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
                    self.scan = scan
                    self.corners = self.findCorners(scan)
                    items = [item for item in scan]
                    distances = [item[2] for item in items]
                    angles = [item[1] for item in items]
                    self.slam.update(distances, angles)
                    sleep(0.001)
            except Exception as e:
                logging.error(f"Lidar error: {e}")
                self.lidar.reset()
                sleep(5)

    def findCorners(self, scan) -> list[tuple]:
        diffList = []
        sumDiff = 0
        for i in range(1, len(scan)):
            diffList.append(scan[i][2] - scan[i-1][2])
            sumDiff += abs(diffList[-1])
        diffList.append(scan[0][2] - scan[-1][2])
        sumDiff += abs(diffList[-1])
        avgDiff = sumDiff / len(diffList)
        cornerList = []
        if abs(diffList[0]) > avgDiff:
            if diffList[0] > 0:
                cornerList.append(scan[-1])
            else:
                cornerList.append(scan[0])
        for i in range(1, len(diffList)):
            if abs(diffList[i]) > avgDiff:
                if diffList[i] > 0:
                    cornerList.append(scan[i - 1])
                else:
                    cornerList.append(scan[i])
        return cornerList

    def stop(self):
        try:
            self.runThread = False
            if not (self.thread is None):
                self.thread.join()
            if not (self.lidar is None):
                self.lidar.stop()
                self.lidar.disconnect() 
        except Exception as e:
            logging.error(f"Lidar Error: {e}")

