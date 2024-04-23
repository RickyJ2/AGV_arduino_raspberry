
import logging
from time import sleep
import serial
import threading
import json

class Arduino:
    def __init__(self, port = '/dev/ttyUSB1', baudrate = 115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        #init variable
        self.container = False
        self.collision = False
        self.orientation = 0 #yaw
        self.acceleration = {
            "x": 0,
            "y": 0,
        }
        self.power = 100

        self.statuspoint = False
    
    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1, dsrdtr=True, rtscts=True)
            sleep(2)
            self.reset()
            logging.info("Arduino connected")
        except serial.SerialException as e:
            logging.error(f"Arduino's connection Failed: {e}")
            sleep(5)

    def reset(self):
        self.ser.dtr = False
        sleep(2)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.ser.dtr = True

    def start(self):
        self.runThread = True
        self.thread_read = threading.Thread(target=self.reader, daemon=True)
        self.thread_read.start()

    def reader(self):
        while True:
            if self.ser is None:
                self.connect()
                continue
            if not self.runThread:
                break
            buffer = ''
            if not (self.ser.in_waiting > 0):
                continue
            try:
                buffer = self.ser.readline().decode("utf-8")
                print(buffer)
                msg = json.loads(buffer)
                if(msg["type"] == "state"):
                    data = msg["data"]
                    self.container = data['container']
                    self.collision = data['collision']
                    self.orientation = data['orientation']
                    self.acceleration = data['acceleration']
                    self.power = data['power']
                elif(msg["type"] == "notif"):
                    self.statuspoint = True
                    print(f"reached point {msg["data"]}")
                else:
                    print(f"Arduino msg: {msg}")
            except Exception as e:
                logging.error(f"Arduino Error: {e} msg: {buffer}")

    def send(self, message):
        self.ser.write(bytes(message, 'utf-8'))

    def close(self):
        try:
            self.runThread = False
            self.thread_read.join()
            if not (self.ser is None):
                self.ser.close()
        except Exception as e:
            logging.error(f"Arduino Error: {e}")

    def getContainer(self):
        return self.container
    
    def getCollision(self):
        return self.collision
    
    def getOrientation(self):
        return self.orientation

    def getAcceleration(self):
        return self.acceleration
    
    def getPower(self):
        return self.power
