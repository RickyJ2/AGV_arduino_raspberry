import math

class Hex:
    def __init__(self, q, r):
        self.q = q
        self.r = r
        self.s = -q - r
        self.walkable = True
    def key(self):
        return '{},{}'.format(self.q, self.r)
    def __eq__(self, other):
        if isinstance(other, Hex):
            return self.q == other.q and self.r == other.r and self.s == other.s
    def __ne__(self, other):
        return not self.__eq__(other)
    def __add__(self, other):
        if isinstance(other, Hex):
            return Hex(self.q + other.q, self.r + other.r)
        raise TypeError("Unsupported operand type for +: 'hex' and '{}'".format(type(other)))
    def __sub__(self, other):
        if isinstance(other, Hex):
            return Hex(self.q - other.q, self.r - other.r)
        raise TypeError("Unsupported operand type for -: 'hex' and '{}'".format(type(other)))
    def __mul__(self, other):
        if isinstance(other, int):
            return Hex(self.q * other, self.r * other)
        raise TypeError("Unsupported operand type for *: 'hex' and '{}'".format(type(other)))
    def __str__(self):
        return "q: {} r: {} s: {}".format(self.q, self.r, self.s)
    def length(self):
        return (abs(self.q) + abs(self.r) + abs(self.s)) / 2
    def distance(self, other):
        return (self - other).length()
    def neighbor(self, direction):
        return self + hexDirections[direction]

hexDirections = [
    Hex(1, 0), Hex(1, -1), Hex(0, -1),
    Hex(-1, 0), Hex(-1, 1), Hex(0, 1),
]

def findDirection(hex):
    for i, direction in enumerate(hexDirections):
        if direction == hex:
            return i
    return -1

def PolarToAxial(distance: float, angle: float, hexagonSize: int) -> tuple:
    x = distance * math.cos(math.radians(angle))
    y = distance * math.sin(math.radians(angle))
    p = math.sqrt(3)/3.0 * x - y/3.0 
    q = 2.0 * y / 3
    p = p / hexagonSize
    q = q / hexagonSize
    r = -p - q
    return (p, q, r)

def AxialToCoord(q: float, r: float, hexagonSize: int) -> tuple:
    x = hexagonSize * (3/2 * q)
    y = hexagonSize * (math.sqrt(3)/2 * q + math.sqrt(3) * r)
    return (x, y)

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