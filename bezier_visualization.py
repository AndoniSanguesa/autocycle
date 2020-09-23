import matplotlib.pyplot as plt
import numpy as np
from matplotlib.font_manager import FontProperties
from pip._vendor.distlib.compat import raw_input
from Assistant import CurveAssistant
import sys
from io import StringIO
import time

writefile = open("WriteData.py")
exec("writefile")

## OPEN TESTDATA FILE. ONLY FOR TESTING UNTIL INTEGRATION ##
datafile = open("TestData", "r")
lines = datafile.readlines()
stdin = sys.stdin

# Time start for testing efficiency
time0 = time.time()

# Makes the graphs look not dumb. Also created array for the graph labels
fontP = FontProperties()
fontP.set_size("small")
labels = []

# Distance to the end of the graph (our max viewing distance)
sys.stdin = StringIO(lines.pop(0))
end_dist = int(input())

# Distances from the bike to the objects
dist_to_edge = []

# Shortest distance from the edges of the objects to the global path
edge_to_path = []

# Lengths of the objects
edge_len = []

# The final x and y values of the Bezier plot
x_vals = []
y_vals = []


def reset_data():
    dist_to_edge.clear()
    edge_to_path.clear()
    edge_len.clear()


def get_data():
    reset_data()
    # Format is numbers with spaces inbetween:
    # num_obst gamma1 dist_to_edge1 edge_to_path1 side_small_edge1 edge_len1 gamma2 ...
    sys.stdin = StringIO(lines.pop(0))
    data = raw_input().split()
    for x in range(int(data[0])):
        dist_to_edge.append(float(data[x * 3 + 1]))
        edge_to_path.append(float(data[x * 3 + 2]))
        edge_len.append(float(data[x * 3 + 3]))


# Creates a CurveAssistant that will allow us to access our data
curveas = CurveAssistant(end_dist)

# The coordinate of the final control point at the end of the graph
lastpoint = curveas.get_last_control_point()


def plot():
    # Plots the obstacles
    for obstacle in curveas.obstacles:
        # Converts the x-y points into the nt points
        points = obstacle.edge_points
        nt_1 = curveas.convert_nt(points[0][0], points[1][0])
        nt_2 = curveas.convert_nt(points[0][1], points[1][1])
        nt_obst = [[nt_1[0], nt_2[0]], [nt_1[1], nt_2[1]]]
        plt.plot(nt_obst[0], nt_obst[1], linewidth=4)

    # Plots the Current Path
    nt_last_point = curveas.convert_nt(lastpoint[0], lastpoint[1])
    plt.plot(
        [0, nt_last_point[0]], [0, nt_last_point[1]], color="green", linestyle="dashed"
    )

    # Plots the global direction
    glob_x = lastpoint[0] * np.cos(np.deg2rad(curveas.glob_angle))
    glob_y = lastpoint[0] * np.sin(np.deg2rad(curveas.glob_angle))
    glob_end_point = curveas.convert_nt(glob_x, glob_y)
    plt.plot(
        [0, glob_end_point[0]],
        [0, glob_end_point[1]],
        color="purple",
        linestyle="dashed",
    )

    # Plots and displays bezier curve
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
        plt.plot(nt_ctr[0], nt_ctr[1], "ro")
        ind += 1
    labels.extend(["Current Path", "Global Path", "Predicted Path", "Control Point"])
    plt.legend(labels, prop=fontP)

    # Sets the plot axis to not be dumb
    plt.axis("square")
    plt.axis([0, nt_last_point[0], 0, nt_last_point[1]])
    plt.show()
    plt.clf()


# Calculates x and y points for the bezier curve
def calculate_curve():
    # Empties the x and y values for the next curve
    x_vals.clear()
    y_vals.clear()

    # This command creates a curve object which allows us to check the characteristics of the curve
    curve = curveas.get_curve()

    # Variables keep track of x/y values and help in determining extrema
    curpoint = [[0]]
    index = 0
    diff = [0, 0]
    ys = [0, 0]

    while curpoint[0][0] < end_dist:
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


def get_bezier_xval(res, xvals, goal):
    return res * (xvals.index(goal))


def create_environment():
    global resolution
    labels.clear()
    if len(lines) == 0:
        return
    resolution = 0.4
    get_data()
    curveas.clear_obstacles()
    # Creates objects
    for x in range(len(edge_len)):
        curveas.create_obstacle(dist_to_edge[x], edge_to_path[x], edge_len[x])
    calculate_curve()
    # Plots obstacles if they intersect the Bezier curve
    ind = 1
    for obstacle in curveas.obstacles:
        # Allows for computation of control points if and only if the curve intersects the object
        labels.append("Object " + str(ind))
        obstacle.next_side(curveas.extrema[0], curveas.extrema[1])
        while obstacle.intersect(x_vals, y_vals):
            obstacle.adjust_gamma()
            calculate_curve()
        ind += 1
    resolution = 0.001
    calculate_curve()
    plot()
    create_environment()


# Drives the whole operation
def run_visualization(glob_angle=0.00):
    global datafile
    global lines
    curveas.glob_angle = glob_angle
    datafile = open("TestData", "r")
    lines = datafile.readlines()
    sys.stdin = StringIO(lines.pop(0))
    input()
    create_environment()
    return curveas

sys.stdin = stdin
run_visualization()
# Takes the final time value and prints the result
time1 = time.time()
#print(time1 - time0)
