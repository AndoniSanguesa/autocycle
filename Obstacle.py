import numpy as np
import math

class Obstacle:
    def __init__(self, dist_to_edge, vel_dir, edge_to_path, edge_len, side, gamma):
        self.dist_to_edge = dist_to_edge
        self.vel_dir = vel_dir
        self.edge_to_path = edge_to_path
        self.edge_len = edge_len
        self.side = side
        self.edge_points = []
        self.control_points = []
        self.gamma = gamma

        self.calculate_obst_points()
        self.calculate_control_point()

    def calculate_obst_points(self):
        ep = np.sqrt(2)*np.sqrt(self.dist_to_edge**2)

        new_ang_rad = (math.pi/4) + np.tan([self.edge_to_path/self.dist_to_edge])*self.side
        x1 = ep/((np.tan([new_ang_rad])[0])+1)
        y1 = -x1 + ep
        diff = np.sqrt((self.edge_len**2)/2)

        x2 = x1 + diff*self.side
        y2 = y1 - diff*self.side

        self.edge_points.append([x1[0], x2[0]])
        self.edge_points.append([y1[0], y2[0]])

    def calculate_control_point(self):
        angle_rad = np.deg2rad(45 + self.gamma*self.side)
        print(self.side)
        ref_point = 0
        b = self.edge_points[0][ref_point] + self.edge_points[1][ref_point]

        x = b/(np.tan(angle_rad)+1)
        y = -x + b

        self.control_points.append([x, y])
        self.control_points.append([x+1, y])
        self.control_points.append([x, y - 1])
        #self.control_points.append([self.edge_points[0][ref_point], self.edge_points[1][ref_point]])

    def get_obst_points(self):
        return self.edge_points

    def get_control_points(self):
        return self.control_points

    def get_dist_to_edge(self):
        return self.dist_to_edge

    def get_vel_dir(self):
        return self.vel_dir

    def get_edge_to_path(self):
        return self.edge_to_path

    def get_edge_len(self):
        return self.edge_len
