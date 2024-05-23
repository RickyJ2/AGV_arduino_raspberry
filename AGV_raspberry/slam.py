from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel

class SLAM:
    def __init__(self, map_size_pixels = 250, map_size_meters = 10):
        self.slam = RMHC_SLAM(LaserModel(), map_size_pixels, map_size_meters)
        self.pose = [0,0,0]
        self.mapbytes = bytearray(map_size_pixels * map_size_pixels)
        self.minSamples = 150
        self.previousDistances = None
        self.previousAngles = None

    def update(self, distances, angles):
        if len(distances) > self.minSamples:
            self.slam.update(distances, scan_angles_degrees=angles)
            self.previousDistances = distances.copy()
            self.previousAngles = angles.copy()
        elif self.previousDistances is not None:
            self.slam.update(self.previousDistances, scan_angles_degrees=self.previousAngles)

    def getMap(self):
        self.slam.getmap(self.mapbytes)
        return self.mapbytes

    def getPos(self):
        return self.slam.getpos() ## x_mm, y_mm, theta