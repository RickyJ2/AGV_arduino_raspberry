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
            return i*60
    return -1
        

def cubeRound(q, r):
    x = q
    z = r
    y = -x - z
    rx = round(x)
    ry = round(y)
    rz = round(z)
    x_diff = abs(rx - x)
    y_diff = abs(ry - y)
    z_diff = abs(rz - z)
    if x_diff > y_diff and x_diff > z_diff:
        rx = -ry - rz
    elif y_diff > z_diff:
        ry = -rx - rz
    return rx, rz
    