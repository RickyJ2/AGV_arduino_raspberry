import logging
from time import sleep
import serial
import threading
import json

class Arduino:
    def __init__(self, port = '/dev/ttyUSB0', baudrate = 9600):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        #init variable
        self.container = False
        self.power = 100

        self.statuspoint = False
    
    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=5)
            sleep(3)
            logging.info("Reseting Arduino")
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            logging.info("Arduino connected")
        except serial.SerialException as e:
            logging.error(f"Arduino's connection Failed: {e}")
            sleep(5)

    def start(self):
        self.runThread = True
        self.thread_read = threading.Thread(target=self.reader, name="Arduino", daemon=True)
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
                sleep(0.1)
                continue
            try:
                buffer = self.ser.readline().decode("utf-8")
                msg = json.loads(buffer, strict=False)
                if(msg["type"] == "state"):
                    data = msg["data"]
                    logging.info(f"arduino state {data}")
                    self.container = data['container']
                    self.power = data['power']
                else:
                    logging.info(f"Arduino msg: {msg}")
            except Exception as e:
                logging.error(f"Arduino Error: {e} msg: {buffer}")
                if not self.ser.is_open:
                    self.connect()

    def send(self, message):
        try:
            logging.info(f"send arduino msg: {message}")
            self.ser.write(bytes(message, 'utf-8'))
        except Exception as e:
            logging.error(f"Arduino send error: {e}")


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
    
    def getPower(self):
        return self.power
