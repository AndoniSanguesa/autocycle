import math

class LineSeg:
    def __init__(self, coord1, coord2):
        xdiff = coord2[0] - coord1[0]
        ydiff = coord2[1] - coord1[1]
        self.length = math.sqrt(xdiff**2 + ydiff**2)
        self.angle = math.degrees(math.atan(ydiff/xdiff))
        self.coords = [coord1, coord2]