import logging
import math
from Class.lyapunovControl import LyapunovControl
from Class.pose import Pose
from Class.util import distance

class SteeringControl:
    def __init__(self, rightMotorModel, leftMotorModel, width, wheelDiameter, errorTolerance):
        self.rightMotorModel  = rightMotorModel
        self.leftMotorModel = leftMotorModel
        self.width = width
        self.wheelDiameter = wheelDiameter
        self.lyapunovControl: LyapunovControl = LyapunovControl(1, 0.6, errorTolerance)
        self.maxRPM = [70, 80]
        self.minRPM = 65
        self.maxRotateRPM = 80
        self.minRotatedRPM = 75
        self.currentVelocity = 0
    
    def saturated(self, leftRPM, rightRPM, min, max) -> tuple[float, float]:
        timesLeft = 1
        timesRight = 1
        if leftRPM < 0:
            timesLeft = -1
            leftRPM *= -1
        if rightRPM < 0:
            timesRight = -1
            rightRPM *= -1
        #upper bounding
        if leftRPM > max and rightRPM > max:
            if leftRPM > rightRPM:
                rightRPM *= max / rightRPM
                leftRPM = max
            else:
                leftRPM *= max / leftRPM
                rightRPM = max
        if leftRPM > max:
            leftRPM = max
        if rightRPM > max:
            rightRPM = max
        #lower bounding
        if leftRPM < min:
            leftRPM = min
        if rightRPM < min:
            rightRPM = min
        return leftRPM * timesLeft, rightRPM * timesRight
    
    def compute(self, currentPoint: Pose, targetPoint: Pose) -> tuple[float, float]:
        v, omega = self.lyapunovControl.compute(currentPoint, targetPoint)
        self.currentVelocity = v
        vL = v - omega*self.width/2 #Linear Velocity mm/s
        vR = v + omega*self.width/2 #Linear Velocity mm/s
        L = vL * 60 / (math.pi * self.wheelDiameter) #Angular Velocity RPM
        R = vR * 60 / (math.pi * self.wheelDiameter) #Angular Velocity RPM
        if v == 0:
            L, R = self.saturated(L, R, self.minRotatedRPM, self.maxRotateRPM)
        else:
            d = distance(currentPoint, targetPoint)
            if d < 3 * self.lyapunovControl.tolerance:
                L, R = self.saturated(L, R, self.minRPM, self.maxRPM[0])
            else:
                L, R = self.saturated(L, R, self.minRPM, self.maxRPM[-1])
        timesL = 1
        timesR = 1
        if L < 0:
            timesL = -1
            L *= -1
        if R < 0:
            timesR = -1
            R *= -1
        LVolt = self.leftMotorModel(L)
        RVolt = self.rightMotorModel(R)
        return LVolt * timesL, RVolt * timesR

    def getVelocity(self) -> float:
        return self.currentVelocity

    