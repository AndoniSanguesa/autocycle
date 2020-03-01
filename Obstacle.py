import numpy as np
import math

class Obstacle:
    def __init__(self, dist_to_edge, vel_dir, edge_to_path, edge_len, side):
        self.dist_to_edge = dist_to_edge
        self.vel_dir = vel_dir
        self.edge_to_path = edge_to_path
        self.edge_len = edge_len
        self.side = side

    def get_obst_points(self):
        ep = np.sqrt(2)*np.sqrt(self.dist_to_edge)

        new_ang_rad = (math.pi/4) + np.tan([self.edge_to_path/self.dist_to_edge])*self.side
        x1 = ep/((np.tan([new_ang_rad])[0])+1)
        y1 = -x1 + ep
        diff = np.sqrt(self.edge_len/2)
        x2 = x1 + diff*self.side
        y2 = y1 - diff*self.side
        return [[x1[0], x2[0]], [y1[0], y2[0]]]


    def get_dist_to_edge(self):
        return self.dist_to_edge

    def get_vel_dir(self):
        return self.vel_dir

    def get_edge_to_path(self):
        return self.edge_to_path

    def get_edge_len(self):
        return self.edge_len

    def get_end_dist(self):
        return self.end_dist
