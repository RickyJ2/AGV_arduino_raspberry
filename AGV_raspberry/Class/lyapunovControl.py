import math

class LyapunovControl:
    def __init__(self, k1, k2, k3, tolerance):
        self.k1 = k1
        self.k2 = k2
        self.k3 = k3
        self.tolerance = tolerance
    
    def compute(self, currentPoint, targetPoint):
        #point: x, y, theta(radians)
        errorX = targetPoint[0] - currentPoint[0]
        errorY = targetPoint[1] - currentPoint[1]
        errorTheta = targetPoint[2] - currentPoint[2]
        if abs(errorX) < self.tolerance and abs(errorY) < self.tolerance and abs(errorTheta) < math.radians(180):
            return 0, 0
        v = self.k1 * (errorX * math.cos(currentPoint[2]) + errorY * math.sin(currentPoint[2]))
        omega = self.k2 * (-errorX * math.sin(currentPoint[2]) + errorY * math.cos(currentPoint[2])) + self.k3 * (errorTheta)
        return v, omega