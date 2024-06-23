import math
from Class.lyapunovControl import LyapunovControl
from Class.pose import Pose

class SteeringControl:
    def __init__(self, rightMotorModel, leftMotorModel, width, wheelDiameter, errorTolerance):
        self.rightMotorModel  = rightMotorModel
        self.leftMotorModel = leftMotorModel
        self.width = width
        self.wheelDiameter = wheelDiameter
        self.lyapunovControl: LyapunovControl = LyapunovControl(1, 8, 3, errorTolerance)
        self.maxRPM = 90
        self.minRPM = 60
        self.currentVelocity = 0
    
    def saturated(self, leftRPM, rightRPM) -> tuple[float, float]:
        if math.floor(leftRPM) == 0 and math.floor(rightRPM) == 0:
            return 0,0
        timesLeft = 1
        timesRight = 1
        if leftRPM < 0:
            timesLeft = -1
            leftRPM *= -1
        if rightRPM < 0:
            timesRight = -1
            rightRPM *= -1
        #upper bounding
        if leftRPM > self.maxRPM and rightRPM > self.maxRPM:
            if leftRPM > rightRPM:
                rightRPM *= self.maxRPM / rightRPM
                leftRPM = self.maxRPM
            else:
                leftRPM *= self.maxRPM / leftRPM
                rightRPM = self.maxRPM
        if leftRPM > self.maxRPM:
            leftRPM = self.maxRPM
        if rightRPM > self.maxRPM:
            rightRPM = self.maxRPM
        #lower bounding
        if leftRPM < self.minRPM:
            leftRPM = self.minRPM
        if rightRPM < self.minRPM:
            rightRPM = self.minRPM
        return leftRPM * timesLeft, rightRPM * timesRight
    
    def compute(self, currentPoint: Pose, targetPoint: Pose) -> tuple[float, float]:
        v, omega = self.lyapunovControl.compute(currentPoint, targetPoint)
        if math.floor(v) == 0  and math.floor(omega) == 0:
            self.currentVelocity = 0
            return 0,0
        self.currentVelocity = v
        vL = v - omega*self.width/2 #Linear Velocity mm/s
        vR = v + omega*self.width/2 #Linear Velocity mm/s
        L = vL * 60 / (math.pi * self.wheelDiameter) #Angular Velocity RPM
        R = vR * 60 / (math.pi * self.wheelDiameter) #Angular Velocity RPM
        L, R = self.saturated(L, R)
        LVolt = self.leftMotorModel(L)
        RVolt = self.rightMotorModel(R)
        return LVolt, RVolt

    def getVelocity(self) -> float:
        return self.currentVelocity

    