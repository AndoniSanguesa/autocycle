import bezier
import numpy as np
import matplotlib.pyplot as plt
from Assistant import CurveAssistant
import time

time0 = time.time()


# The angle to the control point related to the object
gamma = [35, 15]
# Distance to the end of the graph (our max viewing distance)
end_dist = 10
# Distances from the bike to the objects
dist_to_edge = [4, 6]
# Derivatives of velocity
vel_div = [0, 0]
# Shortest distance from the edges of the objects to the global path
edge_to_path = [1, 4.4]
# Sides that the variables above are based on. 1 is above global path, -1 is below.
side_small_edge = [1, 1]
# Lengths of the objects
edge_len = [6, 4.5]

# Creates a CurveAssistant that will allow us to access our data
curveas = CurveAssistant(end_dist)

# Creates objects
for x in range(len(edge_len)):
    curveas.create_obstacle(dist_to_edge[x], vel_div[x], edge_to_path[x], edge_len[x], side_small_edge[x], gamma[x])

# The coordinate of the final control point at the end of the graph
lastpoint = curveas.get_last_control_point()

# The final x and y values of the Bezier plot
x_vals = []
y_vals = []

# The start and end points on the graph (in x values)
start = 0
end = 5

# The resolution dictates how many points are calculated (the lower the better, but slower)
resolution = 0.05

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

calculate_curve()

# Plots obstacles if they intersect the Bezier curve
for obstacle in curveas.obstacles:
    # Converts the x-y points into the nt points
    points = obstacle.edge_points
    nt_1 = curveas.convert_nt(points[0][0], points[1][0])
    nt_2 = curveas.convert_nt(points[0][1], points[1][1])
    nt_obst = [[nt_1[0], nt_2[0]], [nt_1[1], nt_2[1]]]
    plt.plot(nt_obst[0], nt_obst[1], linewidth=4)
    # Allows for computation of control points if and only if the curve intersects the object
    if obstacle.intersect(x_vals, y_vals):
        obstacle.next_side(curveas.extrema[0], curveas.extrema[1])
        calculate_curve()


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
for x in range(len(ctr_points)):
    nt_ctr = curveas.convert_nt(ctr_points[x][0], ctr_points[x][1])
    plt.plot(nt_ctr[0], nt_ctr[1], 'ro')


# Sets the plot axis to not be dumb
plt.axis('square')
plt.axis([0,nt_last_point[0],0,nt_last_point[1]])
plt.show()

time1 = time.time()
print(time1-time0)
