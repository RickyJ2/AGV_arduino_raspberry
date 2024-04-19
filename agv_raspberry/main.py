import logging
from tornado import httpclient
from client import Client
from arduino import Arduino
from lidar import Lidar
import json
import threading
from tornado.ioloop import IOLoop, PeriodicCallback
from hex import Hex, findDirection

IP = "10.53.4.43"
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
# 0 = idle, 1 = pop point,2 = go-to-point 3 = obstacle avoidance, 4 = go-to-nearest point in path
state = 0
runMainThread = False
mainThread = None
ioloop = IOLoop.instance()
lidar = Lidar()
arduino = Arduino()
request = httpclient.HTTPRequest(f"ws://{IP}:{PORT}/agv", headers=header)
client = Client(request, 5)

def clientOnMsg(msg):
    if msg is None:
        return
    msg = json.loads(msg)
    if msg["type"] == "path":
        global pathList, goalPointList
        goalPointList.append(msg["data"]["goal"])
        pathList.append(msg["data"]["path"])
    elif msg["type"] == "stop":
        global state
        state = 2

def sendAGVState():
    data = {
        "container": arduino.getContainer(),
        "collision": arduino.getCollision(),
        "orientation": arduino.getOrientation(),
        "acceleration": arduino.getAcceleration(),
        "power": arduino.getPower(),
        "localMap": lidar.getLocalMap()
    }
    client.send(json.dumps(data))

def main():
    global state, currentGoal, currentPath, goalPointList, pathList, currentCoord, currentTargetPoint
    while True:
        if not runMainThread:
            break
        #if idle state set new goal and path
        if state == 0:
            if len(goalPointList) == 0:
                continue
            #set to new goal point
            state = 1
            currentGoal = goalPointList.pop(0)
            currentPath = pathList.pop(0)
            currentCoord = Hex(0,0)
        elif state == 1:
            #if no path left set state to idle
            if len(currentPath) == 0:
                state = 0
                continue
            #transition to new point in path
            point = currentPath.pop(0)
            currentTargetPoint = Hex(point[0],point[1])
            dir = findDirection(currentTargetPoint - currentCoord)
            data = {
                "direction": dir
            }
            arduino.send(json.dumps(data))
            state = 2
        elif state == 2:
            #collision prediction system and obstacle avoidance
            pass
        elif state == 3:
            #reached target point in path
            currentCoord = currentTargetPoint
            state = 1
            data = {
                "cmd": "stop"
            }
            arduino.send(json.dumps(data))
        elif state == 4:
            pass

def errorHandler():
    global runMainThread
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
        client.connect(clientOnMsg)
        runMainThread = True
        mainThread = threading.Thread(target=main, daemon=True).start()
        PeriodicCallback(sendAGVState, 1000).start()
        ioloop.start()
    except KeyboardInterrupt:
        errorHandler()
    except Exception as e:
        logging.error(f"Error: {e}")
        errorHandler()
    logging.info("Program exit")
