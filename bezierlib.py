import bezier
import numpy as np
import matplotlib.pyplot as plt

# The final x and y values of the Bezier plot
x_vals = []
y_vals = []

# The start and end points on the graph (in x values)
start = 0
end = 0

# The resolution dictates how many points are calculated (the lower the better, but slower)
resolution = 0.01

# List of control points (this should eventually come from the literature)
nodes = np.asfortranarray([
    [0.0, 0.625, 1.0],
    [0.0, 0.5, 0.5]
])

# This command creates a curve object which allows us to check the characteristics of the curve
curve = bezier.Curve(nodes, degree=2)

# This for loop checks individual points and then adds them to the x and y value arrays
for x in np.arange(start, end - 1, resolution):
    curpoint = curve.evaluate(x)
    x_vals.append(curpoint[0][0])
    y_vals.append(curpoint[1][0])

# Plots and displays bezier curve
plt.plot(x_vals, y_vals)
plt.show()
