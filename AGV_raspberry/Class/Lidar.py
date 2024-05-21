from adafruit_rplidar import RPLidar
import serial.tools.list_ports

class Lidar:
    def __init__(self):
        self.lidar = None
        self.max_distance = 0
        self.port = self.find_lidar_port()

    def connect(self):
        try:
            if self.port is None:
                self.port = self.find_lidar_port()
                if self.port is None:
                    raise Exception("Lidar not found")
            self.lidar = RPLidar(None, self.port, timeout=3)
            print(self.lidar.info)
        except Exception as e:
            print(e)

    def find_lidar_port(self):
        ports = list(serial.tools.list_ports.comports())
        lidarPort = None
        for p in ports:
            if p.pid == 60000:
                lidarPort = p.device
        return lidarPort

    def iter_scans(self, max_buf_meas: int = 500, min_len: int = 5):
        scan = []
        iterator = self.lidar.iter_measurements(max_buf_meas)
        for new_scan, quality, angle, distance in iterator:
            if new_scan:
                if len(scan) > min_len:
                    yield scan
                scan = []
            if quality > 0 and distance > 0:
                scan.append((quality, (270 - angle) % 360, distance))

            #     scan[0].append(quality)
            #     scan[1].append(angle)
            #     scan[2].append(distance)
            # else:
            #     scan[0].append(0)
            #     scan[1].append(angle)
            #     scan[2].append(0)

    def stop(self):
        self.lidar.stop()
        self.lidar.disconnect()