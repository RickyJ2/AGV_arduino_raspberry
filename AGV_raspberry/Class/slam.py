from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel

class SLAM:
    def __init__(self, mapSizePixels = 500, mapSizeMeters = 10):
        #configuration
        self.mapSizePixels = mapSizePixels
        self.mapSizeMeters = mapSizeMeters
        self.mapbytes = bytearray(mapSizePixels * mapSizeMeters)
        self.minSamples = 150
        self.init()

    def init(self):
        self.slam = RMHC_SLAM(LaserModel(), self.mapSizePixels, self.mapSizeMeters)
        self.pose = [0,0,0]
        self.previousDistances = None
        self.previousAngles = None

    def update(self, distances: list, angles: list):
        if len(distances) > self.minSamples:
            self.slam.update(distances, scan_angles_degrees=angles)
            self.previousDistances = distances.copy()
            self.previousAngles = angles.copy()
        elif self.previousDistances is not None:
            self.slam.update(self.previousDistances, scan_angles_degrees=self.previousAngles)

    def getPos(self) -> tuple[float, float, float]:
        return self.slam.getpos() ## x_mm, y_mm, theta