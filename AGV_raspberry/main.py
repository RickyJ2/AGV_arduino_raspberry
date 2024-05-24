import logging
from time import sleep
from tornado import httpclient
from client import Client
from arduino import Arduino
from lidar import Lidar
import json
import threading
from tornado.ioloop import IOLoop, PeriodicCallback
from hex import Hex, findDirection
from slam import SLAM

IP = "localhost"
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

def distance(pos, target):
    return ((pos[0] - target[0])**2 + (pos[1] - target[1])**2)**0.5

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
            "localMap": lidar.getLocalMap()
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
                logging.info(f"target: {dir}")
                previousPos = lidar.getPos()
                logging.info("current state will 2")
                state = 2
            elif state == 2:
                pose = lidar.getPos()
                dOrientation = pose[2] - dir
                if dir == 360:
                    dOrientation *= -1
                if abs(dOrientation) < 10 or abs(dOrientation) > 360 - 3:
                    arduino.moveForward()
                    if distance(lidar.getPos(), previousPos) >= 340:
                        arduino.stop()
                        state = 3
                        logging.info("current state will 3")
                        msg = {
                            "type": "notif",
                            "data": "point"
                        }
                        ioloop.add_callback(client.send, json.dumps(msg))
                else:
                    if dir == 360 or dir == 0:
                        if dOrientation > 180:
                            dOrientation -= 360
                    if dOrientation > 0:
                        arduino.moveRight()
                    else:
                        arduino.moveLeft()
            elif state == 3:
                #reached target point in path
                logging.info("in state 3")
                currentCoord = currentTargetPoint
                logging.info("current state will 1")
                state = 1
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
