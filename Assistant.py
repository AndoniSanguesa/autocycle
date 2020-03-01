import numpy as np
from Obstacle import Obstacle


class CurveAssistant:
    def __init__(self, end_dist):
        self.end_dist = end_dist
        self.control_points = []
        self.compute_control_points()
        self.obstacles = []
        self.coordinates = [[], []]

    def obstacle_coordinates(self):
        for obstacle in self.obstacles:
            coord = obstacle.get_obst_points()
            self.coordinates[0].append(coord[0])
            self.coordinates[1].append(coord[1])
        return self.coordinates

    def create_obstacle(self, dist_to_edge, vel_dir, edge_to_path, edge_len, side):
        self.obstacles.append(Obstacle(dist_to_edge, vel_dir, edge_to_path, edge_len, side))

    def get_control_point(self, index):
        return self.control_points[index]

    def get_last_control_point(self):
        return self.get_control_point(len(self.control_points)-1)

    def compute_control_points(self):
        # These lines should always be computed last
        dist = np.sqrt((self.end_dist**2)/2)
        self.control_points.append([dist, dist])

    def get_fortran(self):
        return np.asfortranarray([
            [0, 0, 0, self.control_points[-1][0]],
            [0, 0, 0, self.control_points[-1][1]],
        ])

    def get_end_dist(self):
        return self.end_dist
