import logging
from tornado import httpclient
from client import Client
from arduino import Arduino
from lidar import Lidar
import json
from tornado.ioloop import IOLoop, PeriodicCallback

IP = "10.53.11.85"
PORT = 8080
header = { 
        'websocketpass':'1234', 
        'id':'1'
    }
goalPointList = []
pathList = []
# 0 = idle, 1 = follow path, 2 = obstacle avoidance, 3 = go-to-nearest point in path
state = 0
ioloop = IOLoop.instance()
lidar = Lidar()
arduino = Arduino()
request = httpclient.HTTPRequest(f"ws://{IP}:{PORT}/agv", headers=header)
client = Client(request, 5)

def clientOnMsg(msg):
    if msg is None:
        return
    data = {
        "cmd": msg
    }
    logging.info(f"Sending: {data}")
    arduino.send(json.dumps(data))

def sendAGVState():
    data = {
        "container": arduino.getContainer(),
        "collision": arduino.getCollision(),
        "orientation": arduino.getOrientation(),
        "acceleration": arduino.getAcceleration(),
        "power": arduino.getPower(),
        "lidar": lidar.getScanData()
    }
    client.send(json.dumps(data))

def errorHandler():
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
        PeriodicCallback(sendAGVState, 1000).start()
        ioloop.start()
    except KeyboardInterrupt:
        errorHandler()
    except Exception as e:
        logging.error(f"Error: {e}")
        errorHandler()
    logging.info("Program exit")
