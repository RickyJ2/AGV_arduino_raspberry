import logging
import math
from time import sleep
from tornado import httpclient
from client import Client
from arduino import Arduino
from lidar import Lidar
import json
import threading
from tornado.ioloop import IOLoop, PeriodicCallback
from hex import Hex, findDirection, AxialToCoord
from slam import SLAM
from util import distance, findOrientation
import numpy as np

#CONSTANT
RobotWidth = 189 #mm
WheelDiameter = 36 #mm

IP = "192.168.85.180"
PORT = 8080
header = {
    'websocketpass':'1234',
    'id':'1'
}
goalPointList = []
pathList = []
currentGoal = None
currentPath = []
currentTargetPoint = None
currentCoord = Hex(0,0)
currentDir = 0
# 0 = idle, 1 = pop point,2 = go-to-point 3 = obstacle avoidance, 4 = go-to-nearest point in path
globalCoordinate = Hex(0,0)
state = 0
previousPos = (0,0,0)
runMainThread = False
mainThread = None
ioloop = IOLoop.current()

arduino = Arduino()
slam = SLAM()
lidar = Lidar(slam=slam)
request = httpclient.HTTPRequest(f"ws://{IP}:{PORT}/agv", headers=header)
client = Client(request, 5)

def LyapunovControl(currentPoint, targetPoint):
    #point: x, y, theta(radians)
    errorX = targetPoint[0] - currentPoint[0]
    errorY = targetPoint[1] - currentPoint[1]
    errorTheta = targetPoint[2] - currentPoint[2]
    if abs(errorX) < 350/2 and abs(errorY) < 350/2 and abs(errorTheta) < math.radians(180):
        return 0, 0
    k1 = 1
    k2 = 8
    k3 = 3
    v = k1 * (errorX * math.cos(currentPoint[2]) + errorY * math.sin(currentPoint[2]))
    omega = k2 * (-errorX * math.sin(currentPoint[2]) + errorY * math.cos(currentPoint[2])) + k3 * (errorTheta)
    return v, omega

def steeringControl(currentPoint, targetPoint):
    #point: x (mm), y (mm), theta(radians)
    #return left voltage, right voltage
    v, omega = LyapunovControl(currentPoint, targetPoint)
    if v == 0  and omega == 0:
        return 0,0
    vL = v - omega*RobotWidth/2 #Linear Velocity mm/s
    vR = v + omega*RobotWidth/2 #Linear Velocity mm/s
    L = vL * 60 / (math.pi * WheelDiameter) #Angular Velocity RPM
    R = vR * 60 / (math.pi * WheelDiameter) #Angular Velocity RPM
    if L > 100: 
        L = 100
    if L < 60:
        L = 60
    if R > 100:
        R = 100
    if R < 60:
        R = 60
    LVolt,RVolt = motorModelLeftID01(L), motorModelRightID01(R)
    #LVolt, RVolt = motorModelLeftID02(L), motorModelRightID02(R)
    return LVolt, RVolt

def motorModelRightID01(RPM):
    return np.exp((RPM - 19.52) / 33.90)

def motorModelLeftID01(RPM):
    return np.exp((RPM - 33.53) / 25.64)

def motorModelRightID02(RPM):
    return np.exp((RPM - 25.61) / 28.53)

def motorModelLeftID02(RPM):
    return np.exp((RPM - 39.93) / 24.37)

def clientOnMsg(msg):
    global state, pathList, goalPointList, globalCoordinate
    if msg is None:
        return
    msg = json.loads(msg)
    logging.info(msg)
    if msg["type"] == "position":
        globalCoordinate = Hex(msg["data"]["x"],msg["data"]["y"])
    elif msg["type"] == "path":
        goalPointList.append(msg["data"]["goal"])
        pathList.append(msg["data"]["path"])
    elif msg["type"] == "new path":
        pathList.pop(0)
        #insert on index 0
        pathList.insert(0, msg["data"]["path"])
        logging.info("current state will 1")
        state = 1
    elif msg["type"] == "stop":
        goalPointList = []
        pathList = []
        logging.info("current state will 0")
        state = 0

def sendAGVState():
    msg = {
        "type": "state",
        "data": {
            "container": arduino.getContainer(),
            "power": arduino.getPower(),
            "localMap": lidar.getLocalMap(),
            "orientation": lidar.getPos()[2]
        }
    }
    client.send(json.dumps(msg))

def main():
    global state, currentGoal, currentPath, goalPointList, pathList, currentCoord, currentTargetPoint, currentDir, previousDistance
    while True:
        if not runMainThread:
            break
        try:
            #if idle state set new goal and path
            if state == 0:
                if len(goalPointList) == 0:
                    continue
                #set to new goal point
                state = 1
                logging.info("current state will 1")
                currentGoal = goalPointList.pop(0)
                currentPath = pathList.pop(0)
                currentCoord = Hex(0,0)
            elif state == 1:
                #if no path left set state to idle
                logging.info(f"current path: {currentPath}")
                if len(currentPath) == 0:
                    logging.info("current state will 0")
                    state = 0
                    logging.info(f"current goal list: {goalPointList}")
                    continue
                #transition to new point in path
                point = currentPath.pop(0)
                currentTargetPoint = Hex(point[0],point[1])
                currentDir = findDirection(currentTargetPoint - currentCoord)
                dir = currentDir * 60
                targetPoint = AxialToCoord(currentTargetPoint.q, currentTargetPoint.r, lidar.hexHeight)
                curPos = lidar.getPos()
                Lvolt, Rvolt = steeringControl(curPos, (targetPoint[0], targetPoint[1], math.radians(dir)))
                logging.info(f"target: {dir}")
                data = {
                    "type": "control",
                    "left": Lvolt,
                    "right": Rvolt,
                }
                arduino.send(json.dumps(data))
                logging.info("current state will 2")
                state = 2
            elif state == 2:
                # for i in range(3):
                #     if len(currentPath) <= i:
                #         break
                #     if not lidar.map.getHexAt(currentPath[i][0] - currentCoord[0], currentPath[i][1] - currentCoord[1]).walkable == False:
                #         logging.info("current state will 5")
                #         state = 5
                #         break
                targetPoint = AxialToCoord(currentTargetPoint.q, currentTargetPoint.r, lidar.hexHeight)
                curPos = lidar.getPos()
                logging.error(f"current Position: (${curPos[0]}, ${curPos[1]}) targetPoint: ${targetPoint}")
                Lvolt, Rvolt = steeringControl(curPos, (targetPoint[0], targetPoint[1], math.radians(dir)))
                data = {
                    "type": "control",
                    "left": Lvolt,
                    "right": Rvolt,
                }
                arduino.send(json.dumps(data))
                if Lvolt == 0 and Rvolt == 0:
                    state = 3
                    logging.info("current state will 3")
                    msg = {
                        "type": "notif",
                        "data": "point"
                    }
                    ioloop.add_callback(client.send, json.dumps(msg))
            elif state == 3:
                #reached target point in path
                logging.info("in state 3")
                currentCoord = currentTargetPoint
                logging.info("current state will 1")
                state = 1
            elif state == 4:
                pass
            elif state == 5:
                pass
        except Exception as e:
            logging.error(f"Main Error: {e}")

def errorHandler():
    global runMainThread, mainThread
    runMainThread = False
    mainThread.join()
    ioloop.stop()
    client.closeConnection()
    arduino.close()
    lidar.stop()

if __name__ == "__main__":
    logFormatter = logging.Formatter('[%(levelname)s]\t[%(asctime)s]: %(message)s', datefmt='%d-%m-%Y %H:%M:%S')
    #fileHandler for Logging
    try:
        fileHandler = logging.FileHandler('/home/AGV/python.log')
    except Exception as e:
        fileHandler = logging.FileHandler('python.log')
    fileHandler.setLevel(logging.DEBUG)
    fileHandler.setFormatter(logFormatter)
    #consoleHandler for Logging
    consoleHandler = logging.StreamHandler()
    consoleHandler.setLevel(logging.DEBUG)
    consoleHandler.setFormatter(logFormatter)
    #Configure Logger
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)
    logger.addHandler(fileHandler)
    logger.addHandler(consoleHandler)
    logging.info("Program Start")
    try:
        lidar.start()
        arduino.start()
        sleep(10)
        client.connect(clientOnMsg)
        runMainThread = True
        mainThread = threading.Thread(target=main,name="main", daemon=True)
        mainThread.start()
        PeriodicCallback(sendAGVState, 1000).start()
        ioloop.start()
    except KeyboardInterrupt:
        errorHandler()
    except Exception as e:
        logging.error(f"Error: {e}")
        errorHandler()
    logging.info("Program exit")
