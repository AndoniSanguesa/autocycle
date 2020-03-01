import bezier
import numpy as np
import matplotlib.pyplot as plt
from Assistant import CurveAssistant

# Distance to the end of the graph (our max viewing distance)
end_dist = 10
# Distances from the bike to the objects
dist_to_edge = [4, 10]
# Derivatives of velocity
vel_div = [0, 0]
# Shortest distance from the edges of the objects to the global path
edge_to_path = [2, 2]
# Sides that the variables above are based on. 1 is above global path, -1 is below.
side_small_edge = [-1, 1]
# Lengths of the objects
edge_len = [2, 4]

# Creates a CurveAssistant that will allow us to access our data
curveas = CurveAssistant(end_dist)

# Creates objects
for x in range(len(edge_len)):
    curveas.create_obstacle(dist_to_edge[x], vel_div[x], edge_to_path[x], edge_len[x], side_small_edge[x])

# The coordinate of the final control point at the end of the graph
lastpoint = curveas.get_last_control_point()

# The final x and y values of the Bezier plot
x_vals = []
y_vals = []

# The start and end points on the graph (in x values)
start = 0
end = 5

# The resolution dictates how many points are calculated (the lower the better, but slower)
resolution = 0.01

# List of control points (this should eventually come from the literature)
nodes = curveas.get_fortran()

# This command creates a curve object which allows us to check the characteristics of the curve
curve = bezier.Curve(nodes, degree=3)

# Calculates x and y points for the bezier curve
curpoint = [[0]]
index = 0
while curpoint[0][0] < np.sqrt((end_dist**2)/2):
    curpoint = curve.evaluate(index * 1.00)
    index += resolution
    x_vals.append(curpoint[0][0])
    y_vals.append(curpoint[1][0])

# Plots and displays bezier curve
plt.plot([0, lastpoint[0]], [0, lastpoint[1]], color='green', linestyle='dashed')
plt.plot(x_vals, y_vals)

# The coordinates of the obstacles
obst_points = curveas.obstacle_coordinates()

# Plots the obstacles
for x in range(len(obst_points[0])):
    plt.plot(obst_points[0][x], obst_points[1][x], linewidth=4)

# Sets the plot axis to not be dumb
plt.axis('square')
plt.axis([0,7,0,7])

plt.show()
