import logging
import math
from Class.pose import Pose

class LyapunovControl:
    def __init__(self, k1, k2, tolerance):
        self.k1 = k1
        self.k2 = k2
        self.tolerance = tolerance
    
    def compute(self, currentPoint: Pose, targetPoint: Pose) -> tuple[float, float]:
        error = targetPoint - currentPoint
        v = self.k1 * (error.point.x * math.cos(currentPoint.orientation) 
                       + error.point.y * math.sin(currentPoint.orientation))
        if error.orientation > math.pi:
            error.orientation -= 2 * math.pi
        if error.orientation < -math.pi:
            error.orientation += 2 * math.pi
        omega = self.k2 * error.orientation
        if error.orientation > math.pi/6 or error.orientation < -math.pi/6:
            v = 0
        return v, omega