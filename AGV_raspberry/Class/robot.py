import json
import math
import numpy as np
import logging
from Class.point import Point
from Class.pose import Pose
from Class.slam import SLAM
from Class.arduino import Arduino
from Class.lidar import Lidar
from Class.steeringControl import SteeringControl
from Class.util import distance, findOrientation

def motorModelRightID01(RPM) -> float:
    return np.exp((RPM - 19.52) / 33.90)

def motorModelLeftID01(RPM) -> float:
    return np.exp((RPM - 33.53) / 25.64)

def motorModelRightID02(RPM) -> float:
    return np.exp((RPM - 25.61) / 28.53)

def motorModelLeftID02(RPM) -> float:
    return np.exp((RPM - 39.93) / 24.37)

class Robot:
    def __init__(self, id, state):
        self.id = id
        self.width = 189 #mm
        self.wheelDiameter = 60 #mm
        self.errorTolerance = 60

        self.arduino: Arduino = Arduino()
        self.slam: SLAM = SLAM()
        self.lidar: Lidar = Lidar(slam = self.slam)
        if id == 1:
            self.steeringControl: SteeringControl = SteeringControl(motorModelRightID01, motorModelLeftID01, self.width, self.wheelDiameter, self.errorTolerance)
        elif id == 2:
            self.steeringControl: SteeringControl = SteeringControl(motorModelRightID02, motorModelLeftID02, self.width, self.wheelDiameter, self.errorTolerance)
        
        self.startCoordinate: Point = Point(0,0)
        self.goalPointList: list[Point] = []
        self.pathList: list[list[Point]] = []
        self.currentGoal: Point = None
        self.currentPath: list[Point] = []
        self.currentTargetPose: Pose = None
        self.state = state
        self.clearData()
    
    def clearData(self):
        self.data = {
            "time": [],
            "cur_x": [],
            "cur_y": [],
            "cur_orien": [],
            "goal_x": [],
            "goal_y": [],
            "goal_orien": [],
            "power": [],
            "v_lyapunov": [],
            "omega_lyapunov": [],
            "v_left": [],
            "v_right": [],
        }
    
    def insertData(self, time):
        currentPos = self.getPos()
        goal = self.currentTargetPose
        v = self.steeringControl.getVelocity()
        omega = self.steeringControl.getOmega()
        vLeft = self.steeringControl.leftSpeed
        vRight = self.steeringControl.rightSpeed
        self.data["time"].append(time)
        self.data["cur_x"].append(currentPos.point.x)
        self.data["cur_y"].append(currentPos.point.y)
        self.data["cur_orien"].append(math.degrees(currentPos.orientation))
        self.data["goal_x"].append(goal.point.x)
        self.data["goal_y"].append(goal.point.y)
        self.data["goal_orien"].append(math.degrees(goal.orientation))
        self.data["power"].append(round(self.arduino.power))
        self.data["v_lyapunov"].append(v)
        self.data["omega_lyapunov"].append(omega)
        self.data["v_left"].append(vLeft)
        self.data["v_right"].append(vRight)

    def init(self):
        self.arduino.start()
        self.lidar.start()
    
    def clearFollowPathParams(self):
        self.currentTargetPose = None
        self.currentGoal = None
        self.currentPath = []

    def noGoal(self) -> bool:
        return len(self.goalPointList) == 0

    def updateNewGoal(self):
        self.currentGoal = self.goalPointList.pop(0)
        self.currentPath = self.pathList.pop(0)

    def changeCurrentPath(self, path: list[Point]):
        self.currentPath = path
        self.updateTargetPoint()

    def updateTargetPoint(self):
        if len(self.currentPath) == 0:
            return
        orientation = 0
        if self.currentTargetPose is None:
            orientation = findOrientation(self.currentPath[0], self.currentPath[1])
            self.currentPath.pop(0) #remove the starting point
        else:
            orientation = findOrientation(self.currentTargetPose, self.currentPath[0])
        self.currentTargetPose = Pose(self.currentPath.pop(0), 0)
        self.currentTargetPose.orientation = orientation
        logging.info(f"current Target point: {self.currentTargetPose}")

    def isReachGoal(self) -> bool:
        currentPos: Pose = self.getPos()
        return distance(currentPos, self.currentGoal) < self.errorTolerance
    
    def isReachTargetPoint(self) -> bool:
        if self.currentTargetPose is None:
            return False
        currentPos:Pose = self.getPos()
        return distance(currentPos, self.currentTargetPose) < self.errorTolerance

    def isCurrentPathEmpty(self) -> bool:
        return len(self.currentPath) == 0

    def isCurrentTargetPointNone(self) -> bool:
        return self.currentTargetPose is None

    def stateIs(self, state) -> bool:
        return self.state == state

    def updateState(self, state):
        logging.info(f"State changed from {self.state} to {state}")
        self.state = state

    def stopMoving(self):
        logging.info("Stop moving")
        self.steeringControl.currentVelocity = 0
        data = {
            "type": "control",
            "left": 0,
            "right": 0,
        }
        self.arduino.send(json.dumps(data))

    def steerToTargetPoint(self):
        currentPos = self.getPos()
        self.currentTargetPose.orientation = findOrientation(currentPos, self.currentTargetPose)
        LVolt, RVolt = self.steeringControl.compute(currentPos, self.currentTargetPose)
        data = {
            "type": "control",
            "left": LVolt,
            "right": RVolt,
        }
        self.arduino.send(json.dumps(data))

    def setStartCoordinate(self, point: Point):
        self.startCoordinate = point
    
    def insertGoal(self, goal: Point):
        self.goalPointList.append(goal)

    def insertPath(self, path: list[Point]):
        self.pathList.append(path)

    def getRobotState(self) -> dict:
        pos: Pose = self.getPos()
        return {
            "container": self.arduino.getContainer(),
            "power": self.arduino.getPower(),
            "orientation": math.degrees(pos.orientation),
            "velocity": self.steeringControl.getVelocity(),
            "position": pos.point.toDict(),
        }

    def getPos(self) -> Pose:
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
        lst[0] += 5000 + self.startCoordinate.x
        lst[1] += 5000 + self.startCoordinate.y
        lst[2] = math.radians(orientation)
        return Pose(Point(lst[0], lst[1]), lst[2])

    def convertCornersToGlobal(self, corners: list[tuple], threshold = 700) -> list[Point]:
        currentPos: Pose = self.getPos()
        globalCorners: list[Point] = []
        for _, angle, distance in corners:
            if distance > threshold:
                continue
            angleRad = math.radians(angle) + currentPos.orientation - 90
            x = distance * math.cos(angleRad) + currentPos.point.x
            y = distance * math.sin(angleRad) + currentPos.point.y
            globalCorners.append(Point(x, y))
        return globalCorners
    
    def getCollideObstacle(self) -> list[Point]:
        cornerXY: list[Point] = self.convertCornersToGlobal(self.lidar.scan)
        count = 3 if len(self.currentPath) > 3 else len(self.currentPath)
        collideCorners = []
        for i in range(0, count):
            for corner in cornerXY:
                if distance(corner, self.currentPath[i]) < 175 : #in mm
                    collideCorners.append(corner)
        return collideCorners

    def stop(self):
        self.arduino.close()
        self.lidar.stop()