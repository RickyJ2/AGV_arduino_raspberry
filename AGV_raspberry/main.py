import logging
import json
import math
import threading
from time import sleep
from tornado import httpclient
from Class.client import Client
from tornado.ioloop import IOLoop, PeriodicCallback
from Class.robot import Robot
from config import ID, IP, PORT
from Class.point import Point, dictToPoint
import pygame

#CONSTANTS
IDLE = 0
FOLLOW_PATH = 1
OBSTACLE_AVOIDANCE = 2

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

def sendObs(listObs: list[Point]):
    msg = {
        "type": "obstacle",
        "data": [point.toDict() for point in listObs]
    }
    ioloop.add_callback(client.send, json.dumps(msg)) #For calling in Thread

def main():
    global runMainThread
    while True:
        # pos = agv.getPos()
        # logging.info(f"({pos.point}, {math.degrees(pos.orientation)})")
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
                            agv.updateState(OBSTACLE_AVOIDANCE)
                            continue    
                    agv.updateTargetPoint()
                    sendNotifReachPoint() 
                    if agv.isReachGoal():
                        agv.clearFollowPathParams()
                        agv.updateState(IDLE)
                        continue
                elif agv.isCurrentTargetPointNone():  
                    listObs = agv.getCollideObstacle()
                    if len(listObs) > 0:
                        sendNotifCollided(listObs)
                        agv.stopMoving()
                        agv.updateState(OBSTACLE_AVOIDANCE)
                        continue
                    agv.updateTargetPoint()
                else:
                    agv.steerToTargetPoint()
            elif agv.stateIs(OBSTACLE_AVOIDANCE):
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

def displayLidarReading():
    pygame.init()
    screen = pygame.display.set_mode([600, 600])
    display = True
    while display:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                display = False
        screen.fill((20, 10, 30))
        for _, angle, distance in agv.lidar.scan:
            distance /= 10
            x = 300 + distance * math.sin(math.radians(angle))
            y = 300 + distance * math.cos(math.radians(angle))
            pygame.draw.circle(screen, pygame.Color(255, 0, 0), (x, y), 1)
        # listObs = agv.getCollideObstacle()
        # for point in listObs:
        #     pygame.draw.circle(screen, pygame.Color(0, 255, 0), (300 + point.x/10, 300 + point.y/10), 5)
        for _, angle, distance in agv.lidar.corners:
            distance /= 10
            x = 300 + distance * math.sin(math.radians(angle))
            y = 300 + distance * math.cos(math.radians(angle))
            pygame.draw.circle(screen, pygame.Color(0, 255, 0), (x, y), 5)
        for point, point2 in agv.lidar.lines:
            d1 = point[2] / 10
            d2 = point2[2] / 10
            x1 = 300 + d1 * math.sin(math.radians(point[1]))
            y1 = 300 + d1 * math.cos(math.radians(point[1]))
            x2 = 300 + d2 * math.sin(math.radians(point2[1]))
            y2 = 300 + d2 * math.cos(math.radians(point2[1]))
            pygame.draw.line(screen, pygame.Color(0, 255, 0), (x1, y1), (x2, y2), 1)
        pygame.draw.circle(screen, pygame.Color(100, 100, 120), (300, 300), 5)
        pygame.display.flip()

if __name__ == "__main__":
    configLogger()
    logging.info("Program Start")
    try:
        agv.init()
        sleep(5)
        # displayLidarReading()
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
    logging.info("Program exit")
