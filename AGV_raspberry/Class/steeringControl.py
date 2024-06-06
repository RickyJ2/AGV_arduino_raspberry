import math
from lyapunovControl import LyapunovControl

class SteeringControl:
    def __init__(self, rightMotorModel, leftMotorModel, width, wheelDiameter, errorTolerance):
        self.rightMotorModel = rightMotorModel
        self.leftMotorModel = leftMotorModel
        self.width = width
        self.wheelDiameter = wheelDiameter
        self.lyapunovControl = LyapunovControl(1, 8, 3, errorTolerance)
        self.maxRPM = 100
        self.minRPM = 60
        self.currentVelocity = 0
    
    def saturated(self, val):
        if math.floor(val) == 0:
            return 0
        times = 1
        if val < 0:
            times = -1
            val *= -1
        if val > self.maxRPM:
            val = self.maxRPM
        if val < self.minRPM:
            val = self.minRPM
        return val * times
    
    def compute(self, currentPoint, targetPoint):
        #point: x (mm), y (mm), theta(radians)
        #return left voltage, right voltage
        v, omega = self.lyapunovControl.compute(currentPoint, targetPoint)
        if math.floor(v) == 0  and math.floor(omega) == 0:
            self.currentVelocity = 0
            return 0,0
        self.currentVelocity = v
        vL = v - omega*self.width/2 #Linear Velocity mm/s
        vR = v + omega*self.width/2 #Linear Velocity mm/s
        L = vL * 60 / (math.pi * self.wheelDiameter) #Angular Velocity RPM
        R = vR * 60 / (math.pi * self.wheelDiameter) #Angular Velocity RPM
        L = self.saturated(L)
        R = self.saturated(R)
        LVolt = self.leftMotorModel(L)
        RVolt = self.rightMotorModel(R)
        return LVolt, RVolt

    def getVelocity(self):
        return self.currentVelocity

    