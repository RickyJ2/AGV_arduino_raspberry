import json
import math
import numpy as np
from Class.util import findOrientation, distance
from Class.slam import SLAM
from Class.arduino import Arduino
from Class.lidar import Lidar
from Class.steeringControl import SteeringControl

def motorModelRightID01(RPM):
    return np.exp((RPM - 19.52) / 33.90)

def motorModelLeftID01(RPM):
    return np.exp((RPM - 33.53) / 25.64)

def motorModelRightID02(RPM):
    return np.exp((RPM - 25.61) / 28.53)

def motorModelLeftID02(RPM):
    return np.exp((RPM - 39.93) / 24.37)

class Robot:
    def __init__(self, id, state):
        self.id = id
        self.width = 189 #mm
        self.wheelDiameter = 60 #mm
        self.errorTolerance = 175

        self.arduino: Arduino = Arduino()
        self.slam: SLAM = SLAM()
        self.lidar: Lidar = Lidar(slam = self.slam)
        if id == 1:
            self.steeringControl = SteeringControl(motorModelRightID01, motorModelLeftID01, self.width, self.wheelDiameter, self.errorTolerance)
        elif id == 2:
            self.steeringControl = SteeringControl(motorModelRightID02, motorModelLeftID02, self.width, self.wheelDiameter, self.errorTolerance)
        
        self.startCoordinate: tuple[float, float] = (0,0)
        self.goalPointList = []
        self.pathList = []
        self.currentGoal = None
        self.currentPath = []
        self.currentTargetPoint = None
        self.state = state
    
    def init(self):
        self.arduino.start()
        self.lidar.start()
    
    def clearFollowPathParams(self):
        self.currentTargetPoint = None
        self.currentGoal = None
        self.currentPath = []

    def noGoal(self):
        return len(self.goalPointList) == 0

    def updateNewGoal(self):
        self.currentGoal = self.goalPointList.pop(0)
        self.currentPath = self.pathList.pop(0)

    def updateTargetPoint(self):
        self.currentTargetPoint = self.currentPath.pop(0)
        if len(self.currentPath) == 0:
            self.currentDir = 90
        else:
            self.currentDir = findOrientation(self.currentPath[0] - self.currentTargetPoint)

    def isReachGoal(self):
        currentPos = self.getPos()
        currentPos = (currentPos[0], currentPos[1])
        goal = (self.currentGoal["x"], self.currentGoal["y"])
        return distance(currentPos, goal) < self.errorTolerance
    
    def isReachTargetPoint(self):
        if self.currentTargetPoint is None:
            return False
        currentPos = self.getPos()
        currentPos = (currentPos[0], currentPos[1])
        targetPoint = (self.currentTargetPoint["x"], self.currentTargetPoint["y"])
        return distance(currentPos, targetPoint) < self.errorTolerance

    def isCurrentPathEmpty(self):
        return len(self.currentPath) == 0

    def isCurrentTargetPointNone(self):
        return self.currentTargetPoint is None

    def stateIs(self, state):
        return self.state == state

    def updateState(self, state):
        self.state = state

    def stop(self):
        data = {
            "type": "control",
            "left": 0,
            "right": 0,
        }
        self.arduino.send(json.dumps(data))

    def steerToTargetPoint(self):
        currentPos = self.getPos()
        targetPoint = (self.currentTargetPoint["x"], self.currentTargetPoint["y"], self.currentDir)
        LVolt, RVolt = self.steeringControl.compute(currentPos, targetPoint)
        data = {
            "type": "control",
            "left": LVolt,
            "right": RVolt,
        }
        self.arduino.send(json.dumps(data))

    def setStartCoordinate(self, x, y):
        self.startCoordinate = (x,y)
    
    def insertGoal(self, goal):
        self.goalPointList.append(goal)

    def insertPath(self, path):
        self.pathList.append(path)

    def getRobotState(self):
        pos = self.getPos()
        return {
            "container": self.arduino.getContainer(),
            "power": self.arduino.getPower(),
            "orientation": self.getPos()[2],
            "velocity": self.steeringControl.getVelocity(),
            "position": {
                "x": pos[0],
                "y": pos[1]
            }
        }

    def getPos(self):
        pose = self.slam.getPos()
        orientation = pose[2]
        if orientation > 0:
            orientation += 90
            orientation = orientation % 360
        else:
            orientation = (90 + orientation) * -1
            orientation = orientation % 360
            orientation = 360 - orientation
        lst = list(pose)
        lst[0] *= -1
        lst[1] *= -1
        lst[0] += 5000 + self.startCoordinate[0]
        lst[1] += 5000 + self.startCoordinate[1]
        lst[2] = math.radians(orientation)
        pose = tuple(lst)
        return pose
    
    def stop(self):
        self.arduino.close()
        self.lidar.stop()