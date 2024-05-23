import math
from Class.Lidar import Lidar
from Class.Hexagon import PolarToAxial, HexRound, HexLineDraw
import pygame

HEX_RADIUS = 10

def hex_to_pixel(q, r):
    x = HEX_RADIUS * 3/2 * q
    y = HEX_RADIUS * math.sqrt(3) * (r + q/2)
    return (x, y)

def draw_hexagon(surface, color, center):
    x, y = center
    points = []
    for i in range(6):
        angle = 2 * math.pi / 6 * i
        dx = x + HEX_RADIUS * math.cos(angle)
        dy = y + HEX_RADIUS * math.sin(angle)
        points.append((dx, dy))
    pygame.draw.polygon(surface, color, points)

lidar = Lidar()
lidar.connect()
pygame.init()
WINDOW_WIDTH, WINDOW_HEIGHT = 800, 600
window = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption('Hexagonal Occupancy Grid')

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)

clock = pygame.time.Clock()

occupancy_grid = {
    (0, 0): 0,
}

try:
    for scan in lidar.iter_scans():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                break
        window.fill(WHITE)
        coords = []
        for quality, angle, distance in scan:
            p, q, r = PolarToAxial(distance, angle)
            p, q, r = HexRound(p, q, r)
            coords.append((p,q,r))
        #for every coord in coords draw a line and every hexagon value 0
        for coord in coords:
            occupancy_grid[(coord[0], coord[1])] = 0
            freeGrid = HexLineDraw((0,0,0), coord)
            for free in freeGrid:
                occupancy_grid[(free[0], free[1])] = 1
        #display coords in Hexagon map
        for (q, r), occupied in occupancy_grid.items():
            color = GREEN if occupied else RED
            center = hex_to_pixel(q, r)
            center = (center[0] + WINDOW_WIDTH // 2, center[1] + WINDOW_HEIGHT // 2)
            draw_hexagon(window, color, center)
        pygame.display.flip()
        clock.tick(30)
except KeyboardInterrupt:
    print('Stopping.')
except Exception as e:
    print(e)
finally:
    lidar.stop()
    pygame.quit()