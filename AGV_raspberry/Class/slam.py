from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel

class SLAM:
#Creates a RMHCSlam object suitable for updating with new Lidar and odometry data. 
# laser is a Laser object representing the specifications of your Lidar unit 
# map_size_pixels is the size of the square map in pixels 
# map_size_meters is the size of the square map in meters 
# quality from 0 through 255 determines integration speed of scan into map 
# hole_width_mm determines width of obstacles (walls) 
# random_seed supports reproducible results; defaults to system time if unspecified 
# sigma_xy_mm specifies the standard deviation in millimeters of the normal distribution of the (X,Y) component of position for RMHC search
# sigma_theta_degrees specifies the standard deviation in degrees of the normal distribution of the rotational component of position for RMHC search
# max_search_iter specifies the maximum number of iterations for RMHC search
    def __init__(self, mapSizePixels = 500, mapSizeMeters = 10):
        #configuration
        self.mapSizePixels = mapSizePixels
        self.mapSizeMeters = mapSizeMeters
        self.mapbytes = bytearray(mapSizePixels * mapSizeMeters)
        self.minSamples = 150
        self.init()

    def init(self):
        self.slam = RMHC_SLAM(LaserModel(), self.mapSizePixels, self.mapSizeMeters, map_quality=40, hole_width_mm=2000)
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