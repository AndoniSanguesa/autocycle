import numpy as np
import math

class Obstacle:
    def __init__(self, dist_to_edge, edge_to_path, edge_len, side, gamma):
        self.dist_to_edge = dist_to_edge
        self.edge_to_path = edge_to_path
        self.edge_len = edge_len
        self.side = side
        self.edge_points = []
        self.control_points = []
        self.gamma = gamma
        self.shown = False

        self.calculate_obst_points()
        self.calculate_control_point()

    # Calculates the end points of the obstacle
    def calculate_obst_points(self):
        x = self.dist_to_edge
        y1 = self.side*self.edge_to_path
        y2 = self.side*(self.edge_to_path-self.edge_len)
        self.edge_points.append([x, x])

        # This is structured this way so that the top point is always the first in the list
        if self.side == 1:
            self.edge_points.append([y1, y2])
        else:
            self.edge_points.append([y2, y1])

    # Calculates the control points associated with this obstacle
    def calculate_control_point(self):
        self.control_points.clear()
        angle_rad = np.deg2rad(self.gamma*self.side)
        x = self.dist_to_edge
        y = np.tan(angle_rad)*x

        self.control_points.append([x, y])
        self.control_points.append([x-(0.5*self.side), y-(0.5*self.side)])
        self.control_points.append([x+(0.5*self.side), y - (0.5*self.side)])
        #self.control_points.append([self.edge_points[0][ref_point], self.edge_points[1][ref_point]])

    # Determines if a set of x and y coordinates at any point intersect with the obstacle
    def intersect(self, xs, ys):
        close_x = xs[0]
        for x in xs:
            if abs(self.dist_to_edge - x) < abs(self.dist_to_edge - close_x):
                close_x = x
        close_index = xs.index(close_x)
        close_y = ys[close_index]
        thisys = self.edge_points[1]
        if thisys[0] < close_y < thisys[1] or thisys[1] < close_y < thisys[0]:
            self.shown = True
            return True
        return False

    # Determines which end point of the obstacle is closest to the provided x and y value
    def next_side(self, x, y):
        diff_s1 = (self.edge_points[0][0]-x)**2 + (self.edge_points[1][0]-y)**2
        diff_s2 = (self.edge_points[0][1]-x)**2 + (self.edge_points[1][1]-y)**2
        if diff_s1 <= diff_s2:
            self.side = 1
        else:
            self.side = -1
        self.calculate_control_point()

    def get_obst_points(self):
        return self.edge_points

    def get_control_points(self):
        return self.control_points

    def get_dist_to_edge(self):
        return self.dist_to_edge

    def get_edge_to_path(self):
        return self.edge_to_path

    def get_edge_len(self):
        return self.edge_len
