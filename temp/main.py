import math

import numpy as np
from Class.Lidar import Lidar
from odometry import odometry_lidar_only, dynamic_downsample, resample_scan
import pygame

def PolarToCartesian(distance: float, angle: float) -> tuple:
    x = distance * math.cos(math.radians(angle))
    y = distance * math.sin(math.radians(angle))
    return (x, y)
lidar = Lidar()
lidar.connect()

pose = [0, 0, 0]

pygame.init()
screen = pygame.display.set_mode((600, 600))
screen.fill((255, 255, 255))
pygame.display.flip()

try:
    previousScan = None
    for scan in lidar.iter_scans():
        currentScan = []
        for quality, angle, distance in scan:
            x, y = PolarToCartesian(distance, angle)
            currentScan.append([x, y])
        currentScan = np.array(currentScan)
        currentScan = dynamic_downsample(currentScan)
        currentScan = resample_scan(currentScan)
        for x, y in currentScan:  
            pygame.draw.circle(screen, (0, 0, 0), (int(x) + 300, int(y) + 300), 2)
        pygame.display.flip()
        # if previousScan is None:
        #     previousScan = currentScan
        #     continue
        # x, y, degree = odometry_lidar_only(previousScan, currentScan)
        # pose = [pose[0] + x, pose[1] + y, pose[2] + degree]
        # print(pose)
        # previousScan = currentScan
except KeyboardInterrupt:
    print('Stopping.')
except Exception as e:
    print(e)
finally:
    lidar.stop()