import math
from Class.pose import Pose

class LyapunovControl:
    def __init__(self, k1, k2, k3, tolerance):
        self.k1 = k1
        self.k2 = k2
        self.k3 = k3
        self.tolerance = tolerance
    
    def compute(self, currentPoint: Pose, targetPoint: Pose):
        error = targetPoint - currentPoint
        if abs(error.point.x) < self.tolerance and abs(error.point.y) < self.tolerance and abs(error.orientation) < math.radians(180):
            return 0, 0
        v = self.k1 * (error.point.x * math.cos(currentPoint.orientation) + error.point.y * math.sin(currentPoint.orientation))
        omega = self.k2 * (-error.point.x * math.sin(currentPoint.orientation) + error.point.y * math.cos(currentPoint.orientation)) + self.k3 * (error.orientation)
        return v, omega