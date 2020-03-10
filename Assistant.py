import numpy as np
from Obstacle import Obstacle


class CurveAssistant:
    def __init__(self, end_dist):
        self.end_dist = end_dist
        self.control_points = []
        self.obstacles = []
        self.coordinates = [[], []]
        self.compute_control_points()

    def obstacle_coordinates(self):
        self.coordinates[0].clear()
        self.coordinates[1].clear()
        for obstacle in self.obstacles:
        return self.coordinates

    def create_obstacle(self, dist_to_edge, vel_dir, edge_to_path, edge_len, side, gamma):
        self.obstacles.append(Obstacle(dist_to_edge, vel_dir, edge_to_path, edge_len, side, gamma))
        self.compute_control_points()

    def get_control_points(self):
        return self.control_points

    def get_last_control_point(self):
        return self.get_control_points()[-1]

    def compute_control_points(self):
        self.control_points.clear()
        self.control_points.append([0, 0])
        self.control_points.append([1, 1])
        for obstacle in self.obstacles:

            coord = obstacle.get_control_points()
            self.control_points.extend(coord)

        # These lines should always be computed last
        dist = np.sqrt((self.end_dist**2)/2)
        self.control_points.append([dist, dist])

    def get_num_control_points(self):
        return len(self.control_points)

    def obstacle_intersect(self, obst_num):


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

    def get_end_dist(self):
        return self.end_dist
