import numpy as np
from Obstacle import Obstacle
import math
from LineSeg import LineSeg

# Looks for closest x_val in an array through binary search
def find_closest_x(x_vals, target):
    start = 0
    end = len(x_vals)-1
    while start != end:
        half_ind = start + math.floor((end-start) / 2)
        half_val = x_vals[half_ind]
        if half_val > target:
            end = half_ind-1
        elif half_val < target:
            start = half_ind+1
        else:
            return half_val
    return start

# The CurveAssistant stores all the information for a given curve and
# allows the user to perform calculations using it

class CurveAssistant:
    def __init__(self, end_dist):
        self.end_dist = end_dist
        self.control_points = []
        self.obstacles = []
        self.coordinates = [[], []]
        self.compute_control_points()
        self.extrema = [0, 0]

    # Updates obstacle coordinates contained in the curve graph
    def obstacle_coordinates(self):
        # Clears the coordinates for recalculation
        self.coordinates[0].clear()
        self.coordinates[1].clear()

        # Gets each set of coordinates and stores them
        for obstacle in self.obstacles:
            coord = obstacle.get_obst_points()
            self.coordinates[0].append(coord[0])
            self.coordinates[1].append(coord[1])
        return self.coordinates

    # Creates obstacle object and produces initial control points
    def create_obstacle(self, dist_to_edge, edge_to_path, edge_len, side, gamma):
        self.obstacles.append(Obstacle(dist_to_edge, edge_to_path, edge_len, side, gamma))
        self.compute_control_points()

    def clear_obstacles(self):
        self.obstacles.clear()

    # Returns control Points
    def get_control_points(self):
        self.compute_control_points()
        return self.control_points

    # Returns the last control point
    def get_last_control_point(self):
        return self.get_control_points()[-1]

    # Updates the control Points
    def compute_control_points(self):
        # Clears control points
        self.control_points.clear()
        self.control_points.append([0, 0])
        self.control_points.append([1, 0])

        # Adds all control points associated to each obstacle
        for obstacle in self.obstacles:
            if obstacle.shown:
                coord = obstacle.get_control_points()
                self.control_points.extend(coord)

        # These lines should always be computed last
        self.control_points.append([self.end_dist, 0])

    def get_line_seg(self, xvals, yvals, target):
        close_ind = find_closest_x(xvals, target)
        coord1 = [xvals[close_ind], yvals[close_ind]]
        coord2 = [xvals[close_ind+1], yvals[close_ind+1]]
        return LineSeg(coord1, coord2)

    # Returns the number of control points
    def get_num_control_points(self):
        return len(self.control_points)

    # Returns the fortran array needed by the bezier library
    def get_fortran(self):
        xs = []
        ys = []
        for point in self.control_points:
            xs.append(point[0])
            ys.append(point[1])
        return np.asfortranarray([
            xs,
            ys,
        ])

    # Converts an xy point into an nt point
    def convert_nt(self, x, y):
        scal = np.sqrt(2)/2
        x1 = scal*(x-y)
        y1 = scal*(x+y)
        return [x1, y1]

    # Returns the end distance
    def get_end_dist(self):
        return self.end_dist
