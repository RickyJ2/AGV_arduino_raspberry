import logging
import json
import threading
from time import sleep
import time
import pandas as pd
from datetime import datetime
import uuid
import os
from tornado import httpclient
from Class.client import Client
from tornado.ioloop import IOLoop, PeriodicCallback
from Class.robot import Robot
from config import ID, IP, PORT, FILENAME
from Class.point import Point, dictToPoint

#CONSTANTS
IDLE = 0
FOLLOW_PATH = 1
WAIT_PATH = 2

header = {
    'id': f'{ID}'
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
        agv.setStartCoordinate(dictToPoint(data))
    elif type == "path":
        path = map(dictToPoint, data["path"])
        agv.insertGoal(dictToPoint(data["goal"]))
        agv.insertPath(list(path))
    elif type == "newPath":
        path = map(dictToPoint, data["path"])
        agv.changeCurrentPath(list(path))
        agv.updateState(FOLLOW_PATH)

def sendAGVState():
    msg = {
        "type": "state",
        "data": agv.getRobotState()
    }
    client.send(json.dumps(msg))

def sendNotifReachPoint():
    logging.info("Send Notif")
    msg = {
        "type": "notif",
    }
    ioloop.add_callback(client.send, json.dumps(msg)) #For calling in Thread

def sendNotifCollided(listObs: list[Point]):
    msg = {
        "type": "collision",
        "data": [point.toDict() for point in listObs]
    }
    ioloop.add_callback(client.send, json.dumps(msg)) #For calling in Thread

def writeToExcel(data):
    if len(data["time"]) == 0:
        logging.info("No data to write")
        return
    df = pd.DataFrame(data)
    timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
    unique_uuid = uuid.uuid4().hex[:6]
    sheet_name = f"AGV{ID}_{timestamp}_{unique_uuid}"
    if not os.path.exists(FILENAME):
        with pd.ExcelWriter(FILENAME, mode='w', engine='openpyxl') as writer:
            df.to_excel(writer, sheet_name=sheet_name, index=False)
        logging.info(f"Data has been written to {FILENAME} sheet {sheet_name}")
    else:
        with pd.ExcelWriter(FILENAME, mode='a', engine='openpyxl') as writer:
            df.to_excel(writer, sheet_name=sheet_name, index=False)
        logging.info(f"Data has been written to {FILENAME} sheet {sheet_name}")

def main():
    global runMainThread
    previousTime = time.time()
    previousSampleTime = time.time()
    while True:
        if not runMainThread:
            break
        try:
            if agv.stateIs(IDLE):
                if agv.noGoal():
                    if agv.steeringControl.currentVelocity != 0:
                        agv.stopMoving()
                    continue
                #set to new goal point
                agv.updateState(FOLLOW_PATH)
                agv.updateNewGoal()
            elif agv.stateIs(FOLLOW_PATH):
                if agv.isReachTargetPoint():
                    agv.stopMoving()
                    if not agv.isCurrentPathEmpty():
                        listObs = agv.getCollideObstacle()
                        if len(listObs) > 0:
                            sendNotifCollided(listObs)
                            agv.stopMoving()
                            agv.updateState(WAIT_PATH)
                            continue    
                    agv.updateTargetPoint()
                    sendNotifReachPoint()
                    writeToExcel(agv.data)
                    agv.clearData()
                    if agv.isReachGoal():
                        agv.clearFollowPathParams()
                        agv.updateState(IDLE)
                    continue
                elif agv.isCurrentTargetPointNone():  
                    listObs = agv.getCollideObstacle()
                    if len(listObs) > 0:
                        sendNotifCollided(listObs)
                        agv.stopMoving()
                        agv.updateState(WAIT_PATH)
                        continue
                    agv.updateTargetPoint()
                    continue
                elif (time.time() - previousTime)*1000 > 50:
                    previousTime = time.time()
                    agv.steerToTargetPoint()
                if (time.time() - previousSampleTime)*1000 > 100:
                    previousSampleTime = time.time()
                    agv.insertData(datetime.now().strftime("%H:%M:%S.%f"))
            elif agv.stateIs(WAIT_PATH):
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
        sleep(5)
        runMainThread = True
        mainThread = threading.Thread(target=main, name="Main", daemon=True)
        mainThread.start()
        sleep(10)
        logging.info("Finish starting up...")
        client.connect(clientOnMsg)
        PeriodicCallback(sendAGVState, 1000).start()
        ioloop.start()
    except KeyboardInterrupt:
        errorHandler()
    except Exception as e:
        logging.error(f"Error: {e}")
        errorHandler()
    writeToExcel(agv.data)
    logging.info("Program exit")
