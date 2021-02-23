import bezier
import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.font_manager import FontProperties
from pip._vendor.distlib.compat import raw_input
import sys
from io import StringIO
import time
import rospy
from autocycle.srv import ObjectList, ObjectListResponse
from autocycle.msg import Curve

class Obstacle:
    def __init__(self, dist_to_edge, edge_to_path, edge_len):
        self.dist_to_edge = dist_to_edge
        self.edge_to_path = edge_to_path
        self.edge_len = edge_len
        self.edge_points = []
        self.control_points = []
        self.shown = False
        if edge_to_path <= edge_len/2:
            self.side = 1
        else:
            self.side = -1
            self.edge_to_path = edge_len-edge_to_path
        self.gamma = math.degrees(math.atan(self.edge_to_path / self.dist_to_edge)) + 5


        self.calculate_obst_points()
        self.calculate_control_point()

    def adjust_gamma(self):
        self.gamma += 5
        self.calculate_control_point()

    # Calculates the end points of the obstacle
    def calculate_obst_points(self):
        self.edge_points.clear()
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
        # self.control_points.append([self.edge_points[0][ref_point], self.edge_points[1][ref_point]])

    # Determines if a set of x and y coordinates at any point intersect with the obstacle
    def intersect(self, xs, ys):
        close_index = find_closest_x(xs, self.dist_to_edge)
        close_y = ys[close_index]
        thisys = self.edge_points[1]
        if thisys[0]-0.1 < close_y < thisys[1]+0.1 or thisys[1]-0.1 < close_y < thisys[0]+0.1:
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

# Looks for closest x_val in an array through binary search
def find_closest_x(x_vals, target):
    start = 0
    end = len(x_vals)-1
    while start != end:
        half_ind = start + math.floor((end-start) / 2)
        half_val = x_vals[half_ind]
        if end == start+1:
            if abs(x_vals[end]-target) > abs(x_vals[start]-target):
                return start
            else:
                return end
        if half_val > target:
            end = half_ind-1
        elif half_val < target:
            start = half_ind+1
        else:
            return half_ind
    return start

# The CurveAssistant stores all the information for a given curve and
# allows the user to perform calculations using it

class CurveAssistant:
    def __init__(self, end_dist):
        self.end_dist = end_dist
        self.control_points = []
        self.obstacles = []
        self.coordinates = [[], []]
        self.heading = 0
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

    def get_curve(self):
        self.compute_control_points()
        nodes = self.get_fortran()
        return bezier.Curve(nodes, self.get_num_control_points() - 1)

    # Creates obstacle object and produces initial control points
    def create_obstacle(self, dist_to_edge, edge_to_path, edge_len):
        self.obstacles.append(Obstacle(dist_to_edge, edge_to_path, edge_len))
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
        self.control_points.append([np.cos(self.heading)*self.end_dist, np.sin(self.heading)*self.end_dist])

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

resolution = 0.04

## OPEN TESTDATA FILE. ONLY FOR TESTING UNTIL INTEGRATION ##
time0 = time.time()
fontP = FontProperties()
fontP.set_size('small')
labels = []
# Distance to the end of the graph (our max viewing distance)
end_dist = 10
# Distances from the bike to the objects
dist_to_edge = []
# Shortest distance from the edges of the objects to the global path
edge_to_path = []
# Lengths of the objects
edge_len = []

# Desired heading
des_heading = 0

# The final x and y values of the Bezier plot
x_vals = []
y_vals = []

# Creates a CurveAssistant that will allow us to access our data
curveas = CurveAssistant(end_dist)

# The coordinate of the final control point at the end of the graph
lastpoint = curveas.get_last_control_point()


def plot():
    for obstacle in curveas.obstacles:
        # Converts the x-y points into the nt points
        points = obstacle.edge_points
        nt_1 = curveas.convert_nt(points[0][0], points[1][0])
        nt_2 = curveas.convert_nt(points[0][1], points[1][1])
        nt_obst = [[nt_1[0], nt_2[0]], [nt_1[1], nt_2[1]]]
        plt.plot(nt_obst[0], nt_obst[1], linewidth=4)

    # Plots and displays bezier curve
    nt_last_point = curveas.convert_nt(lastpoint[0], lastpoint[1])
    plt.plot([0, nt_last_point[0]], [0, nt_last_point[1]], color='green', linestyle='dashed')
    nt_bez_list = [[], []]
    for index in range(len(x_vals)):
        # Converts xy points to nt points for the bezier curve
        nt_bez_cur = curveas.convert_nt(x_vals[index], y_vals[index])
        nt_bez_list[0].append(nt_bez_cur[0])
        nt_bez_list[1].append(nt_bez_cur[1])

    # Plots Bezier Curve
    plt.plot(nt_bez_list[0], nt_bez_list[1])
    # Plots Control points after converting from xy to nt
    ctr_points = curveas.get_control_points()
    ind = 0
    for x in range(len(ctr_points)):
        nt_ctr = curveas.convert_nt(ctr_points[x][0], ctr_points[x][1])
        plt.plot(nt_ctr[0], nt_ctr[1], 'ro')
        ind += 1
    labels.extend(['Global Path', 'Actual Path', 'Control Point'])
    plt.legend(labels, prop=fontP)

    # Sets the plot axis to not be dumb
    plt.axis('square')
    plt.axis([0, nt_last_point[0], 0, nt_last_point[1]])
    plt.show()
    plt.clf()


# Calculates x and y points for the bezier curve
def calculate_curve():
    # Empties the x and y values for the next curve
    x_vals.clear()
    y_vals.clear()

    # The coordinates of the obstacles
    curveas.compute_control_points()

    # List of control points
    nodes = curveas.get_fortran()

    # This command creates a curve object which allows us to check the characteristics of the curve
    curve = bezier.Curve(nodes, curveas.get_num_control_points() - 1)

    # Variables keep track of x/y values and help in determining extrema
    curpoint = [[0]]
    index = 0
    diff = [0, 0]
    ys = [0, 0]

    while index < 1:
        curpoint = curve.evaluate(index * 1.00)
        x = curpoint[0][0]
        y = curpoint[1][0]

        # To determine where the extrema occurs on the bezier curve
        ys[0] = ys[1]
        ys[1] = y
        diff[0] = diff[1]
        diff[1] = ys[1] - ys[0]
        if diff[1] * diff[0] < 0:
            curveas.extrema = [x, y]

        # Adds x and y values to their respective array
        x_vals.append(curpoint[0][0])
        y_vals.append(curpoint[1][0])
        index += resolution


def is_obstacle_block():
    """
    :returns a list of obstacles which currently intersect the path"""
    obst_block = []
    for obstacle in curveas.obstacles:
        if obstacle.intersect(x_vals, y_vals):
            obst_block.append(obstacle)
    return obst_block


def create_environment(req):
    global resolution, des_heading
    
    ## Resets obstacle list for the current curve object
    curveas.obstacles = []

    ## Creates the Service Client that will get speed data
    data_getter = rospy.ServiceProxy("get_data", GetData)
    
    heading = des_heading - data_getter("heading").data
    curveas.heading = heading

    pub = rospy.Publisher('cycle/curve', Curve, queue_size=1)

    for object in req.obj_lst:
        dist_to_edge.append(object.dist_to_edge - req.distance)
        edge_to_path.append(object.edge_to_path)
        edge_len.append(object.edge_len)
    rospy.loginfo("Object data accepted. Generating Path")
    resolution = 0.05
    # Creates objects
    for x in range(len(edge_len)):
        curveas.create_obstacle(dist_to_edge[x], edge_to_path[x], edge_len[x])
    calculate_curve()
    # Plots obstacles if they intersect the Bezier curve
    ind = 1
    count = 0
    lim = 1000
    for obstacle in curveas.obstacles:
        # Allows for computation of control points if and only if the curve intersects the object
        labels.append("Object " + str(ind))
        obstacle.next_side(curveas.extrema[0], curveas.extrema[1])
        ind += 1
    block_list = is_obstacle_block()
    while block_list:
        for obstacle in block_list:
            obstacle.adjust_gamma()
        calculate_curve()
        count += 1
        if count > lim:
            break
        block_list = is_obstacle_block()
    rospy.loginfo("Path generated.")
    pub.publish(str(curveas.get_curve().to_symbolic()), curveas.get_curve().length)
    data_getter.close()
    pub.close()
    return


def start():
    global des_heading
    # Initialize the node and register it with the master.
    rospy.init_node("bezier")

    
    
    ## Sets desired heading (for now the intial heading)
    des_heading = data_getter("heading").data

    rospy.spin()


