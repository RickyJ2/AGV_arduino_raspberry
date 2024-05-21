import math

hexagonSize = 100 #in mm

#reference "SLAM on the Hexagonal Grid" by Piotr Duszak, 2022
def PolarToAxial(distance: float, angle: float) -> tuple:
    p = distance * math.cos(math.radians(angle))
    q = distance * math.cos(math.radians(120) - math.radians(angle))
    r = distance * math.cos(math.radians(240) - math.radians(angle))
    p = p / hexagonSize
    q = q / hexagonSize
    r = r / hexagonSize
    return (p, q, r)

def PolarToCartesian(distance: float, angle: float) -> tuple:
    x = distance * math.cos(math.radians(angle))
    y = distance * math.sin(math.radians(angle))
    return (x, y)

#reference "Geometric Transformations on the Hexagonal Grid" by Innchyn Her, 1995
def HexRound(p: float, q: float, r: float) -> tuple:
    x = round(p)
    y = round(q)
    z = round(r)
    xd = abs(x - p)
    yd = abs(y - q)
    zd = abs(z - r)

    if xd > yd and xd > zd:
        x = -y - z
    elif yd > xd and yd > zd:
        y = -x - z
    else:
        z = -x - y
    return (x, y, z)

def lerp(a, b, t):
    return a + (b - a) * t

def HexLerp(a, b, t):
    p1, q1, r1 = a
    p2, q2, r2 = b
    return lerp(p1, p2, t), lerp(q1, q2, t), lerp(r1, r2, t)

def hexDistance(a, b):
    p1, q1, r1 = a
    p2, q2, r2 = b
    return (abs(p1 - p2) + abs(q1 - q2) + abs(r1 - r2)) / 2

def HexLineDraw(start, end):
    N = round(hexDistance(start, end))
    results = []
    step = 1.0/ max(N, 1)
    for i in range(0, N):
        p, q, r = HexLerp(start, end, i * step)
        results.append(HexRound(p, q, r))
    return results