import math

def distance(pos, target):
    return math.sqrt((pos[0] - target[0])**2 + (pos[1] - target[1])**2)

def findOrientation(pos, target):
    return math.degrees(math.atan2(target[1] - pos[1], target[0] - pos[0]))