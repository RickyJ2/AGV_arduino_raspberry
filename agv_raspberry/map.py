from hex import Hex, hexDirections

class Map:
    def __init__(self, size = 3):
        self.grid = {}
        self.size = size
        self.createGrid()
    def createGrid(self):
        for q in range(-self.size, self.size + 1):
            r1 = max(-self.size, -self.size - q)
            r2 = min(self.size, self.size - q)
            for r in range(r1, r2 + 1):
                hex = Hex(q, r)
                self.grid[hex.key()] = hex
    def clearMap(self):
        for hex in self.grid.values():
            hex.walkable = True
    def getHexAt(self, q, r):
        hex = Hex(q, r)
        return self.grid.get(hex.key())
    def getHexByKey(self, key):
        return self.grid.get(key)
    def addObstacle(self, q, r):
        hex = self.getHexAt(q, r)
        if hex is not None:
            hex.walkable = False
    def getObstacles(self):
        obstacles = []
        for hex in self.grid.values():
            if not hex.walkable:
                obstacles.append(hex)
        return obstacles
    def getWalkableNeighbors(self, hex):
        neighbors = []
        for direction in hexDirections:
            neighbor = hex.neighbor(direction)
            if self.grid.get(neighbor.key()) is not None and self.grid.get(neighbor.key()).walkable:
                neighbors.append(neighbor)
        return neighbors

