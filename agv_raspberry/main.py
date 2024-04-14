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
ioloop = IOLoop.instance()

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
    try:
        #Start Lidar
        lidar = Lidar()
        lidar.start()

        #Serial communication to Arduino
        arduino = Arduino()
        arduino.start()

        #Websocket communication to server
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
        client.connect(clientOnMsg)

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
        PeriodicCallback(sendAGVState, 1000).start()
        ioloop.start()
    except KeyboardInterrupt:
        ioloop.stop()
        client.closeConnection()
        arduino.close()
        lidar.stop()
    except Exception as e:
        logging.error(f"Error: {e}")
        ioloop.stop()
        client.closeConnection()
        arduino.close()
        lidar.stop()
    logging.info("Program exit")
