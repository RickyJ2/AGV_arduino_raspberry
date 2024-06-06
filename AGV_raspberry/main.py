import logging
import json
import threading
from time import sleep
from tornado import httpclient
from Class.client import Client
from tornado.ioloop import IOLoop, PeriodicCallback
from Class.robot import Robot
from config import ID, IP, PORT

#CONSTANTS
IDLE = 0
FOLLOW_PATH = 1
OBSTACLE_AVOIDANCE = 2
GO_TO_NEAREST_POINT = 3

header = {
    'id': ID
}
agv = Robot(ID, IDLE)
ioloop = IOLoop.current()
request = httpclient.HTTPRequest(f"ws://{IP}:{PORT}/agv", headers=header)
client = Client(request, 5)

runMainThread = False
mainThread = None

def clientOnMsg(msg):
    if msg is None:
        return
    msg = json.loads(msg)
    logging.info(msg)
    type = msg["type"]
    data = msg["data"]
    if type == "position":
        agv.setStartCoordinate(data["x"],data["y"])
    elif type == "path":
        agv.insertGoal(data["goal"])
        agv.insertPath(data["path"])

def sendAGVState():
    msg = {
        "type": "state",
        "data": agv.getRobotState()
    }
    client.send(json.dumps(msg))

def main():
    global runMainThread
    while True:
        if not runMainThread:
            break
        try:
            if agv.stateIs(IDLE):
                if agv.noGoal():
                    continue
                #set to new goal point
                agv.updateState(FOLLOW_PATH)
                logging.info("current state will be FOLLOW_PATH")
                agv.updateNewGoal()
            elif agv.stateIs(FOLLOW_PATH):
                if agv.isReachGoal():
                    agv.clearFollowPathParams()
                    logging.info("current state will be IDLE")
                    agv.updateState(IDLE)
                    continue
                if agv.isReachTargetPoint():
                    agv.updateTargetPoint()
                    msg = {
                        "type": "notif",
                        "data": "point"
                    }
                    ioloop.add_callback(client.send, json.dumps(msg))
                else:
                    agv.steerToTargetPoint()
            elif agv.stateIs(OBSTACLE_AVOIDANCE):
                pass
            elif agv.stateIs(GO_TO_NEAREST_POINT):
                pass
        except Exception as e:
            logging.error(f"Main Error: {e}")

def errorHandler():
    global runMainThread, mainThread
    runMainThread = False
    if mainThread is not None:
        mainThread.join()
    ioloop.stop()
    agv.stop()
    client.closeConnection()

def configLogger():
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

if __name__ == "__main__":
    configLogger()
    logging.info("Program Start")
    try:
        agv.init()
        sleep(10)
        runMainThread = True
        mainThread = threading.Thread(target=main, name="Main", daemon=True)
        mainThread.start()
        client.connect(clientOnMsg)
        PeriodicCallback(sendAGVState, 1000).start()
        ioloop.start()
    except KeyboardInterrupt:
        errorHandler()
    except Exception as e:
        logging.error(f"Error: {e}")
        errorHandler()
    logging.info("Program exit")
