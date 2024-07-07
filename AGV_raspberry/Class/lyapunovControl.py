import math
from Class.pose import Pose

class LyapunovControl:
    def __init__(self, k1, k2, tolerance):
        self.k1 = k1
        self.k2 = k2
        self.tolerance = tolerance
    
    def compute(self, currentPoint: Pose, targetPoint: Pose) -> tuple[float, float]:
        error = targetPoint - currentPoint
        if abs(error.point.x) < self.tolerance and abs(error.point.y) < self.tolerance and abs(error.orientation) < math.radians(180):
            return 0, 0
        v = self.k1 * (error.point.x * math.cos(currentPoint.orientation) + error.point.y * math.sin(currentPoint.orientation))
        omega = -self.k2 * error.orientation

        if error.orientation > math.pi:
            error.orientation -= 2 * math.pi
        if error.orientation < -math.pi:
            error.orientation += 2 * math.pi
        if error.orientation > math.pi/2 or error.orientation < -math.pi/2:
            v = 0
        
        return v, omega