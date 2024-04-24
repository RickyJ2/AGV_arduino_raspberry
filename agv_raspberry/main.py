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
from adafruit_rplidar import RPLidarException
from hex import Hex, findDirection, hexDirections
import serial.tools.list_ports

IP = "10.53.12.19"
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
targetLandMark = []
# 0 = idle, 1 = pop point,2 = go-to-point 3 = obstacle avoidance, 4 = go-to-nearest point in path
globalCoordinate = Hex(0,0)
state = 0
runMainThread = False
mainThread = None
ioloop = IOLoop.current()

ports = list(serial.tools.list_ports.comports())
arduinoPort = ""
lidarPort = ""
for p in ports:
    if p.description == "USB Serial":
        arduinoPort = p.device
    else:   
        lidarPort = p.device
arduino = Arduino(port=arduinoPort)
lidar = Lidar(arduino=arduino, port=lidarPort)
request = httpclient.HTTPRequest(f"ws://{IP}:{PORT}/agv", headers=header)
client = Client(request, 5)

def clientOnMsg(msg):
    if msg is None:
        return
    msg = json.loads(msg)
    logging.info(msg)
    if msg["type"] == "position":
        global globalCoordinate
        globalCoordinate = Hex(msg["data"]["x"],msg["data"]["y"])
    elif msg["type"] == "path":
        global pathList, goalPointList
        goalPointList.append(msg["data"]["goal"])
        pathList.append(msg["data"]["path"])
    elif msg["type"] == "stop":
        global state
        logging.info("current state will 3")
        state = 3

def sendAGVState():
    msg = {
        "type": "state",
        "data": {
            "container": arduino.getContainer(),
            "collision": arduino.getCollision(),
            "orientation": arduino.getOrientation(),
            "acceleration": arduino.getAcceleration(),
            "power": arduino.getPower(),
            "localMap": lidar.getLocalMap()
        }
    }
    client.send(json.dumps(msg))

def lidarScan():
    while True:
        if lidar.lidar is None:
            lidar.connect()
            continue
        if not lidar.runThread:
            break
        try:
            for scan in lidar.lidar.iter_scans():
                temp = [0] * 360
                if not lidar.runThread:
                    break
                for _, angle, distance in scan:
                    ang = (270 - (math.floor(angle))) % 360 
                    robotOrientation = 90 - arduino.getOrientation()
                    if robotOrientation < 90:
                        ang = (360 + ang - robotOrientation) % 360
                    else:
                        ang = (ang + robotOrientation) % 360
                    if distance > lidar.max_distance: 
                        temp[ang] = lidar.max_distance
                        continue
                    elif distance < lidar.min_distance:
                        temp[ang] = 0
                        continue
                    temp[ang] = distance
                lidar.res_scan = temp
                lidar.convertToHex()
        except RPLidarException as e:
            logging.error(f"Lidar error: {e}")
            lidar.lidar.reset()
            sleep(5)

def main():
    global state, currentGoal, currentPath, goalPointList, pathList, currentCoord, currentTargetPoint, targetLandMark, currentDir
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
                targetLandMark = lidar.map.getObstacles()
                currentTargetPoint = Hex(point[0],point[1])
                currentDir = findDirection(currentTargetPoint - currentCoord)
                for i in range(len(targetLandMark)):
                    targetLandMark[i] = targetLandMark[i] -  hexDirections[currentDir]
                dir = currentDir * 60
                logging.info(f"target: {dir}")
                data = {
                    "type": "direction",
                    "direction": dir
                }
                arduino.send(json.dumps(data))
                logging.info("current state will 2")
                # logging.info(f"front distance {lidar.res_scan[90]}, back distance {lidar.res_scan[270]}")
                state = 2
            elif state == 2:
                #collision prediction system and obstacle avoidance
                if arduino.statuspoint:
                    arduino.statuspoint = False
                    state = 3
                    logging.info("current state will 3")
                    msg = {
                        "type": "notif",
                        "data": "point"
                    }
                    ioloop.add_callback(client.send, json.dumps(msg))
                # client.send(json.dumps(msg))
                # counter = 0
                # for landmark in targetLandMark:
                #     temp = lidar.map.getHexByKey(landmark.key())
                #     if temp is not None:
                #         if temp.walkable == False:
                #             counter += 1
                # if counter/len(targetLandMark) > 0.9:
                #     msg = {
                #         "type": "notif",
                #         "data": "point"
                #     }
                #     client.send(json.dumps(msg))
            elif state == 3:
                #reached target point in path
                logging.info("in state 3")
                # logging.info(f"front distance {lidar.res_scan[90]}, back distance {lidar.res_scan[270]}")
                currentCoord = currentTargetPoint
                logging.info("current state will 1")
                state = 1
                # data = {
                #     "type": "cmd",
                #     "cmd": "stop"
                # }
                # arduino.send(json.dumps(data))
            elif state == 4:
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
    fileHandler = logging.FileHandler('/home/AGV/python.log')
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
        lidar.start(scan=lidarScan)
        arduino.start()
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
