import logging
from time import sleep
import serial
import threading
import json
import serial.tools.list_ports

class Arduino:
    def __init__(self, baudrate = 9600):
        self.port = None
        self.baudrate = baudrate
        self.ser: serial.Serial = None
        #init variable
        self.container: bool = False
        self.power = 100
    
    def connect(self):
        try:
            if self.port is None:
                if not self.find_arduino_port():
                    raise Exception("Arduino not found")
            self.ser = serial.Serial(self.port, self.baudrate, timeout=5)
            sleep(3)
            logging.info("Reseting Arduino")
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            logging.info("Arduino connected")
        except serial.SerialException as e:
            logging.error(f"Arduino's connection Failed: {e}")
            sleep(5)

    def find_arduino_port(self) -> bool:
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            if p.pid == 29987:
                self.port = p.device
                return True
        return False

    def start(self):
        self.runThread = True
        self.thread_read = threading.Thread(target=self.reader, name="Arduino", daemon=True)
        self.thread_read.start()

    def reader(self):
        while True:
            if not self.runThread:
                break
            if self.ser is None:
                self.connect()
                continue
            buffer = ''
            if not (self.ser.in_waiting > 0):
                sleep(0.1)
                continue
            try:
                buffer = self.ser.readline().decode("utf-8")
                msg = json.loads(buffer, strict=False)
                if(msg["type"] == "state"):
                    data = msg["data"]
                    # logging.info(f"arduino state {data}")
                    self.container = data['container']
                    self.power = data['power']
                else:
                    logging.debug(f"Arduino msg: {msg}")
            except Exception as e:
                logging.error(f"Arduino Error: {e} msg: {buffer}")
                if not self.ser.is_open:
                    self.connect()

    def send(self, message):
        try:
            # logging.info(f"Arduino send: {message}")
            self.ser.write(bytes(message, 'utf-8'))
            self.ser.write(bytes('\n', 'utf-8')) #endline
        except Exception as e:
            logging.error(f"Arduino send error: {e}")

    def close(self):
        try:
            self.runThread = False
            if not (self.thread_read is None):
                self.thread_read.join()
            if not (self.ser is None):
                self.ser.close()
        except Exception as e:
            logging.error(f"Arduino Error: {e}")

    def getContainer(self) -> bool:
        return self.container
    
    def getPower(self) -> int:
        return self.power